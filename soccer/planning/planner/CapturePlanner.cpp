#include "CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "Geometry2d/Pose.hpp"
#include "Geometry2d/Rect.hpp"
#include <functional>
#include <mutex>
#include "motion/TrapezoidalMotion.hpp"
#include "omp.h"

namespace Planning {
using namespace Geometry2d;

REGISTER_CONFIGURABLE(CapturePlanner);

ConfigDouble* CapturePlanner::_maxApproachAngle;
ConfigDouble* CapturePlanner::_ballContactAccelPercent;

void CapturePlanner::createConfiguration(Configuration* cfg) {
    _maxApproachAngle = new ConfigDouble(cfg,
        "Capture/maxApproachAngle", 0);
    _ballContactAccelPercent = new ConfigDouble(cfg,
        "Capture/ballContactAccelPercent", 0);
}
RobotInstant CapturePlanner::getGoalInstant(const PlanRequest& request) {
    auto bruteForceResult = bruteForceCapture(request);
    if(bruteForceResult) {
        return RobotInstant{std::get<1>(*bruteForceResult), {}, RJ::Time(0s)};
    }
    Point ballPoint = request.context->state.ball.pos;
    Point startPoint = request.start.pose.position();
    return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
}
Trajectory CapturePlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant) {
    Trajectory prevTrajectoryCopy = request.prevTrajectory;
    Trajectory newTrajectory = partialReplan(std::move(request), goalInstant);
    if(newTrajectory.end_time() < prevTrajectoryCopy.end_time()) {
        return std::move(newTrajectory);
    }
    return std::move(prevTrajectoryCopy);
}
Trajectory CapturePlanner::partialReplan(PlanRequest&& request, RobotInstant goalInstant) {
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
    if(partial_post.empty()) {
        return reuse(std::move(request));
    }
    Trajectory combo_path{std::move(partial_pre), std::move(partial_post)};
    std::optional<double> angle_override =
            request.context->robot_intents[request.shellID].angle_override;
    if(angle_override) {
        PlanAngles(combo_path, combo_path.first(),
                AngleFns::faceAngle(*angle_override), request.constraints.rot);
    }
    return std::move(combo_path);
}
Trajectory CapturePlanner::fullReplan(PlanRequest&& request, RobotInstant goalInstant) {
    RJ::Time contactTime = request.context->state.ball.estimateTimeTo(goalInstant.pose.position());
    contactTime = std::min(contactTime, RJ::Time{RJ::now() + RJ::Seconds{.1s}});
    auto captureResult = attemptCapture(request, contactTime);
    if(!captureResult) {
        return reuse(std::move(request));
    }
    Trajectory new_path = std::move(std::get<0>(*captureResult));
    if(new_path.empty()) {
        return reuse(std::move(request));
    }
    std::optional<double> angle_override =
            request.context->robot_intents[request.shellID].angle_override;
    if(angle_override) {
        PlanAngles(new_path, new_path.first(),
                   AngleFns::faceAngle(*angle_override), request.constraints.rot);
    }
    return std::move(new_path);
}
//todo(motion planning) find a better way to handle moving targets
std::optional<std::tuple<Trajectory, Pose>> CapturePlanner::bruteForceCapture(const PlanRequest& request) const {
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
    auto [ballHorizonPos, ballHorizonVel, ballHorizonTime, ballHorizOutOfBounds]
    = predictFutureBallState(ball,RJ::now() + RJ::Seconds{1000s});
    std::vector<RJ::Time> times;
    double searchEnd = std::min(maxSearchDuration, RJ::Seconds{ballHorizonTime-RJ::now()}.count());
    double searchStart = std::min(timeEstimate, searchEnd);
    for(int i = 0; i < iterations; i++) {
        double percent = (double)i / (iterations-1);
        times.push_back(RJ::now() + RJ::Seconds{searchStart + percent
        * (searchEnd-searchStart)});
    }
    // Run brute force using a parallel for loop thanks to OpenMP
    // considering the interceptBufferTime constraint:
    // if the constraint can be satisfied, find the path of minimum contact time
    // if not, find the path with maximum intercept buffer time
    Trajectory path{{}};
    RJ::Time bestTime = RJ::Time::max();
    Pose contactPose;
#pragma omp parallel for default(none) shared(contactPose, path, request, \
times, ball, bestTime)
    for(int i = 0; i < iterations; i++) {
        auto pathResult = attemptCapture(request, times[i]);
#pragma omp critical
        if(pathResult) {
            auto& [candidatePath, candContactPose, successfulCapture] = *pathResult;
            if (!candidatePath.empty()) {
                double ballOffset = Robot_MouthRadius + Ball_Radius;
                Point ballPoint = candContactPose.position() + Point::direction(
                        candContactPose.heading()) * (ballOffset);
                RJ::Time candContactTime = ball.estimateTimeTo(ballPoint);
                if(candContactTime < bestTime) {
                    bestTime = candContactTime;
                    path = std::move(candidatePath);
                    contactPose = candContactPose;
                }
            }
        }
    }
    printf("brute force took %.3f sec\n", RJ::Seconds(RJ::now()-startTime).count());//todo(Ethan) delete
    if(!path.empty()) {
        return std::make_tuple(std::move(path), contactPose);
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
    bool outOfBounds = false;
    if (!fieldRect.containsPoint(futureBallPoint)) {
        outOfBounds = true;
        futureBallPoint = projectPointIntoField(futureBallPoint, fieldRect,
                                                ball.pos);
        contactTime = ball.estimateTimeTo(futureBallPoint, &futureBallPoint);
    }
    return std::make_tuple(futureBallPoint, futureBallVel, contactTime, outOfBounds);
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
}