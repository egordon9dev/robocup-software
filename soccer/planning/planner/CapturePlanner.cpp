#include "CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "Geometry2d/Pose.hpp"
#include "Geometry2d/Rect.hpp"
#include <functional>
#include <mutex>
#include "motion/TrapezoidalMotion.hpp"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(CapturePlanner);

ConfigDouble* CapturePlanner::_searchStartDist;
ConfigDouble* CapturePlanner::_searchEndDist;
ConfigDouble* CapturePlanner::_searchIncDist;
ConfigDouble* CapturePlanner::_maxBallSpeedDirect;
ConfigDouble* CapturePlanner::_maxApproachAngle;
ConfigDouble* CapturePlanner::_maxOutrunBallSpeedPercent;
ConfigDouble* CapturePlanner::_ballContactAccelPercent;
ConfigDouble* CapturePlanner::_collectBufferDistBeforeContact;
ConfigDouble* CapturePlanner::_collectBufferDistAfterContact;
ConfigDouble* CapturePlanner::_settleBufferTimeBeforeContact;
ConfigDouble* CapturePlanner::_touchDeltaSpeed;
ConfigDouble* CapturePlanner::_ballSpeedPercentForDampen;
ConfigDouble* CapturePlanner::_collectBallSpeedApproachDirectionCutoff;
ConfigDouble* CapturePlanner::_settleTargetSensitivity;

void CapturePlanner::createConfiguration(Configuration* cfg) {
    _searchStartDist = new ConfigDouble(cfg,
        "Capture/searchStartDist",0);
    _searchEndDist = new ConfigDouble(cfg,
        "Capture/searchEndDist",0);
    _searchIncDist = new ConfigDouble(cfg,
        "Capture/searchIncDist",0);
    _maxBallSpeedDirect = new ConfigDouble(cfg,
        "Capture/maxBallSpeedDirect", 0);
    _maxOutrunBallSpeedPercent = new ConfigDouble(cfg,
        "Capture/maxOutrunBallSpeed", 0);
    _maxApproachAngle = new ConfigDouble(cfg,
        "Capture/maxApproachAngle", 0);
    _ballContactAccelPercent = new ConfigDouble(cfg,
        "Capture/ballContactAccelPercent", 0);
    _collectBufferDistBeforeContact = new ConfigDouble(cfg,
                                                       "Capture/Collect/bufferDistBeforeContact", 0);
    _collectBufferDistAfterContact = new ConfigDouble(cfg,
                                                      "Capture/Collect/bufferDistAfterContact", 0);
    _settleBufferTimeBeforeContact = new ConfigDouble(cfg,
            "Capture/Settle/bufferTimeBeforeContact", 0);
    _touchDeltaSpeed = new ConfigDouble(cfg,
        "Capture/touchDeltaSpeed", 0);
    _ballSpeedPercentForDampen = new ConfigDouble(cfg,
            "Capture/ballSpeedPercentForDampen", 0);
    _collectBallSpeedApproachDirectionCutoff = new ConfigDouble(cfg,
            "Capture/Collect/ballSpeedApproachDirectionCutoff",0);
    _settleTargetSensitivity = new ConfigDouble(cfg,
            "Capture/Settle/targetSensitivity", 0);
}

Trajectory CapturePlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    Trajectory prevTrajectoryCopy = request.prevTrajectory;
    Trajectory newTrajectory = partialReplan(std::move(request), goalInstant, angleFunction);
    if(newTrajectory.end_time() < prevTrajectoryCopy.end_time()) {
        return std::move(newTrajectory);
    }
    return std::move(prevTrajectoryCopy);
}
Trajectory CapturePlanner::partialReplan(PlanRequest&& request, RobotInstant goalInstant,   AngleFunction angleFunction) {
    RJ::Time contactTime = request.context->state.ball.estimateTimeTo(goalInstant.pose.position());
    contactTime = std::min(contactTime, RJ::Time{RJ::now() + RJ::Seconds{20s}});
    RobotInstant startInstant = request.start;
    Trajectory partial_pre = partialPath(request.prevTrajectory);
    request.start = partial_pre.last();
    auto captureResult = attemptCapture(request, contactTime);
    if(!captureResult) {
        return reuse(std::move(request));
    }
    Trajectory partial_post = std::move(std::get<0>(*captureResult));
    return Trajectory{std::move(partial_pre), std::move(partial_post)};
}
Trajectory CapturePlanner::fullReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    RJ::Time contactTime = request.context->state.ball.estimateTimeTo(goalInstant.pose.position());
    contactTime = std::min(contactTime, RJ::Time{RJ::now() + RJ::Seconds{.1s}});
    auto captureResult = attemptCapture(request, contactTime);
    if(!captureResult) {
        return reuse(std::move(request));
    }
    return std::move(std::get<0>(*captureResult));
}
//todo(motion planning) find a better way to handle moving targets
std::optional<std::tuple<Trajectory, Geometry2d::Pose>> CapturePlanner::bruteForceCapture(const PlanRequest& request) const {
    Ball& ball = request.context->state.ball;
    int its = 0;
    constexpr double maxSearchDuration = 6;
    constexpr int iterations = 30;
    RJ::Time startTime = RJ::now();
    //Calculate the time values to use for brute force
    Point botPos = request.start.pose.position();
    double botSpeed = request.start.velocity.linear().mag();
    double distToBallLine = std::abs(ball.vel.norm().cross(ball.pos-botPos));
    const double timeEstimate = Trapezoidal::getTime(distToBallLine, distToBallLine, request.constraints.mot.maxSpeed,request.constraints.mot.maxAcceleration, botSpeed, 0);
    auto [ballHorizonPos, ballHorizonVel, ballHorizonTime, ballHorizonFake]
    = predictFutureBallState(ball,RJ::now() + RJ::Seconds{1000s});
    std::vector<RJ::Time> times;
    double searchEnd = std::min(maxSearchDuration, RJ::Seconds{ballHorizonTime-RJ::now()}.count());
    double searchStart = std::min(timeEstimate, searchEnd);
    for(int i = 0; i < iterations; i++) {
        double percent = (double)i / (iterations-1);
        times.push_back(RJ::now() + RJ::Seconds{searchStart + percent
        * (searchEnd-searchStart)});
    }
    //Run brute force using a parallel for loop thanks to OpenMP
    std::optional<Trajectory> path;
    Geometry2d::Pose contactPose;
    #pragma omp parallel for default(none) shared(contactPose, path, request, times)
    for(int i = 0; i < iterations; i++) {
        auto pathResult = attemptCapture(request, times[i]);
        #pragma omp critical
        if(pathResult) {
            auto [candidatePath, contactPose, successfulCapture] = std::move(*pathResult);
            if (!candidatePath.empty() && (!path || candidatePath.duration() < path->duration())) {
                path = std::move(candidatePath);
            }
        }
    }
    printf("brute force took %.3f sec\n", RJ::Seconds(RJ::now()-startTime).count());//todo(Ethan) delete
    if(path) {
        assert(!path->empty());
        return std::make_tuple(std::move(*path), contactPose);
    }
    //sometimes RRT fails, so use the old path and try again next iteration
    return std::nullopt;
}

std::tuple<Point, Point, RJ::Time, bool> CapturePlanner::predictFutureBallState(const Ball& ball, RJ::Time contactTime) const {
    // Consider the current state of the ball, predict the future state of the ball
    const Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    MotionInstant futureBallInstant = ball.predict(contactTime);
    Point futureBallPoint = futureBallInstant.pos;
    Point futureBallVel = futureBallInstant.vel;
    bool fake = false;
    if (!fieldRect.containsPoint(futureBallPoint)) {
        fake = true;
        futureBallPoint = projectPointIntoField(futureBallPoint, fieldRect,
                                                ball.pos);
        contactTime = ball.estimateTimeTo(futureBallPoint, &futureBallPoint);
    }
    return std::make_tuple(futureBallPoint, futureBallVel, contactTime, fake);
}

Point CapturePlanner::projectPointIntoField(Point targetPoint, const Rect& fieldRect, Point ballPoint) const {
    auto intersectReturn = fieldRect.intersects(Segment(ballPoint, targetPoint));

    bool validIntersect = std::get<0>(intersectReturn);
    std::vector<Point> intersectPts = std::get<1>(intersectReturn);

    // If the ball intersects the field at some point
    // Just get the intersect point as the new target
    if (validIntersect) {
        // Sorts based on distance to intercept target
        // The closest one is the intercept point which the ball moves
        // through leaving the field Not the one on the other side of the
        // field
        // Choose a point just inside the field
        targetPoint = *std::min_element(intersectPts.begin(), intersectPts.end(), [&](Point a, Point b) {
            return (a - targetPoint).mag() <
                   (b - targetPoint).mag();
        });

        // Doesn't intersect
        // project the ball into the field
    } else {
        // Simple projection
        targetPoint.x() = std::clamp(targetPoint.x(),
                                     (double)fieldRect.minx(), (double)fieldRect.maxx());
        targetPoint.y() = std::clamp(targetPoint.y(),
                                     (double)fieldRect.miny(), (double)fieldRect.maxy());
    }
    return targetPoint;
}
//todo(Ethan) add ball as a dyn obs
//TODO this function is too long
std::optional<std::tuple<Trajectory, Pose, bool>> CapturePlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    Point futureBallPoint, futureBallVel;
    bool fakeFutureBall = false;
    std::tie(futureBallPoint, futureBallVel,contactTime,
            fakeFutureBall) = predictFutureBallState(ball, contactTime);

    ShapeSet static_obstacles_with_ball = static_obstacles;

    // Calculate the desired state of the robot at the point of ball contact
    // based on the command type
    Point targetFacePoint;
    double contactSpeed = 0;
    double bufferDistBeforeContact = 0;
    if(std::holds_alternative<CollectCommand>(request.motionCommand)) {
        if(futureBallVel.mag() < *_collectBallSpeedApproachDirectionCutoff) {
            targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
        } else {
            targetFacePoint = ball.pos + futureBallVel.normalized(10);
        }
        contactSpeed = futureBallVel.mag() + *_touchDeltaSpeed;
        bufferDistBeforeContact = *_collectBufferDistBeforeContact;
        static_obstacles_with_ball.add(std::make_shared<Circle>(futureBallPoint, Robot_Radius + Ball_Radius));
    } else if (std::holds_alternative<SettleCommand>(request.motionCommand)) {
        if(futureBallVel.mag() < *_collectBallSpeedApproachDirectionCutoff) {
            targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
        } else {
            targetFacePoint = ball.pos - futureBallVel.normalized(10);
        }
        contactSpeed = -futureBallVel.mag() * *_ballSpeedPercentForDampen;
        double timeUntilContact = RJ::Seconds{contactTime-RJ::now()}.count();
        if(!fakeFutureBall && *_settleBufferTimeBeforeContact > 0 &&
         timeUntilContact > *_settleBufferTimeBeforeContact ) {
            bool bufferFake = false;
            Point bufferBallPt, bufferBallVel;
            RJ::Time bufferTime = contactTime + RJ::Seconds(*_settleBufferTimeBeforeContact);
            std::tie(bufferBallPt, bufferBallVel, bufferTime, bufferFake) = predictFutureBallState(ball, bufferTime);
            if(!bufferFake) {
                bufferDistBeforeContact = (bufferBallPt - futureBallPoint).mag();
            }
        }
        static_obstacles_with_ball.add(std::make_shared<Circle>(ball.pos, Robot_Radius + Ball_Radius));
    } else if (std::holds_alternative<LineKickCommand>(request.motionCommand)) {
        auto lineKickCommand = std::get<LineKickCommand>(request.motionCommand);
        targetFacePoint = lineKickCommand.target;
        contactSpeed = lineKickApproachSpeed;
        bufferDistBeforeContact = *_collectBufferDistBeforeContact;
    } else {
        debugThrow("Invalid Command Type for Capture");
    }

    Point contactFaceDir;
    if(targetFacePoint.distTo(futureBallPoint) < 1e-9) {
        contactFaceDir = (futureBallPoint - startInstant.pose.position()).norm();
    } else {
        contactFaceDir = (targetFacePoint - futureBallPoint).norm();
    }
    //todo(change contactDir during linekick for a one-touch?
    Point contactDir = contactFaceDir;

    Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * contactFaceDir;
    Pose contactPose{contactPoint, contactFaceDir.angle()};
    contactSpeed = std::clamp(contactSpeed, -constraints.mot.maxSpeed, constraints.mot.maxSpeed);

    if(contactDir.angleBetween(contactFaceDir) > *_maxApproachAngle) {
        debugThrow("Attack Angle in Capture too big");
        return std::nullopt;
    }

    // Course: approach the ball in line with the ball velocity
    std::optional<Trajectory> coursePath;
    Point courseTargetPoint = contactPoint - contactDir.normalized(bufferDistBeforeContact);
    Pose courseTargetPose{courseTargetPoint, 0};
    Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
    Point botToContact = contactPoint-startInstant.pose.position();
    // robot is in line with the ball and close to it
    bool inlineWithBall = std::abs(botToContact.cross(contactDir)) < Robot_Radius/2.0;
    bool closeToBall = std::abs(botToContact.mag()) < *_collectBufferDistBeforeContact;
    RJ::Time contactBallTime = RJ::Time::max();
    //if we aren't in the fine segment
    if(ball.pos.distTo(startInstant.pose.position()) > bufferDistBeforeContact + Robot_Radius + Ball_Radius) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles_with_ball, dynamic_obstacles);
        if(coursePath->empty()) {
            printf("attemptCapture: Course RRT Failed\n");
            return std::nullopt;
        }
        courseTargetInstant.stamp = coursePath->last().stamp;
        contactBallTime = coursePath->last().stamp;
    } else {
        printf("attemptCapture: in Fine, returning nullopt\n");
        //todo(Ethan) Really? (see below todo)
        return std::nullopt;
    }

    //Fine: constant velocity while contacting the ball
    RobotConstraints fineConstraints = constraints;
    fineConstraints.mot.maxAcceleration *= *_ballContactAccelPercent;
    fineConstraints.mot.maxSpeed = std::abs(contactSpeed);
    std::optional<Trajectory> finePathBeforeContact, finePathAfterContact;
    // if we plan to stop before making contact with the ball (e.g. a Settle)
    // then we only do a course approach, and wait for the ball to roll into us
    if(fineConstraints.mot.maxSpeed > 1e-6) {
        //Fine Path Before Contact
        Twist contactTwist{contactDir.normalized(contactSpeed), 0};
        RobotInstant contactInstant{contactPose, contactTwist, RJ::now()};
        bool isBeforeContact = coursePath || botToContact.mag() > Robot_MouthRadius + Ball_Radius;
        if(isBeforeContact) {
            RobotInstant instantBeforeFine = coursePath ? courseTargetInstant : startInstant;
            finePathBeforeContact = RRTTrajectory(instantBeforeFine, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathBeforeContact->empty()) {
                if(coursePath) {
                    Trajectory out = std::move(*coursePath);
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    printf("attemptCapture: Fine failed returning Course part\n");
                    return std::make_tuple(std::move(out), contactPose, false);
                }
                printf("attemptCapture: both RRTs Failed\n");
                return std::nullopt;
            }
            contactInstant.stamp = finePathBeforeContact->last().stamp;
        }

        contactBallTime = contactInstant.stamp;
        //Fine Path After Contact
        if(isBeforeContact) {
            double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
            Point fineTargetPoint = contactPoint + contactDir.normalized(*_collectBufferDistAfterContact + stoppingDist);
            RobotInstant fineTargetInstant{Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
            finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
            if(finePathAfterContact->empty()) {
                if(coursePath && finePathBeforeContact) {
                    Trajectory out{std::move(*coursePath), std::move(*finePathBeforeContact)};
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    printf("attemptCapture: finePathAfter failed\n");
                    return std::make_tuple(std::move(out), contactPose, false);
                } else if(coursePath) {
                    Trajectory out = std::move(*coursePath);
                    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                    printf("attemptCapture: Fine before failed\n");
                    return std::make_tuple(std::move(out), contactPose, false);
                }
                printf("attemptCapture: Course and After failed\n");
                return std::nullopt;
            }
        }
    } else {
        if(coursePath) {
            // this is the Main Success Scenario for a Settle Command
            Trajectory out = std::move(*coursePath);
            PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
//            printf("attemptCapture: contactVel=0, using just Course\n");
            return std::make_tuple(std::move(out), contactPose, false);
        } else {
            printf("attemptCapture: contactVel=0, Course RRT Failed\n");
            return std::nullopt;
        }
    }

    //combine all the trajectory segments to build the output path
    Trajectory collectPath{{}};
    if(coursePath) {
        // this is the Main Success Scenario for a Collect Command
//        printf("attemptCapture: Success, returning full path\n");
        assert(finePathBeforeContact && finePathAfterContact);
        Trajectory finePath{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
        collectPath = Trajectory{std::move(*coursePath), std::move(finePath)};
        collectPath.setDebugText("Course");
//        } else if (finePathBeforeContact) {
//            assert(finePathAfterContact);
//            collectPath = Trajectory{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
//            collectPath.setDebugText("Fine");
    } else {
        printf("attemptCapture: too close to ball\n");
        //we're too close to the ball to trust anything. just reuse old path
        //todo(Ethan) REeally?
        return std::nullopt;
    }
    assert(!collectPath.empty());
    bool successfulCapture = contactBallTime - 1e-6s < contactTime;
    PlanAngles(collectPath, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
    return std::make_tuple(std::move(collectPath), contactPose, successfulCapture);
}
}