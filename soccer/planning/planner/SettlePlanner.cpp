#include "SettlePlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {
ConfigDouble* SettlePlanner::_goalPosChangeThreshold;
ConfigDouble* SettlePlanner::_goalVelChangeThreshold;
ConfigDouble* SettlePlanner::_ballSpeedPercentForDampen;
ConfigDouble* SettlePlanner::_bufferTimeBeforeContact;
ConfigDouble* SettlePlanner::_targetSensitivity;

REGISTER_CONFIGURABLE(SettlePlanner);

void SettlePlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg,
            "Capture/Settle/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg,
            "Capture/Settle/goalVelChangeThreshold");
    _bufferTimeBeforeContact = new ConfigDouble(cfg,
                                                "Capture/Settle/bufferTimeBeforeContact", 0);
    _ballSpeedPercentForDampen = new ConfigDouble(cfg,
            "Capture/ballSpeedPercentForDampen", 0);
    _targetSensitivity = new ConfigDouble(cfg,
                                          "Capture/Settle/targetSensitivity", 0);
}
using namespace Geometry2d;
RobotInstant SettlePlanner::getGoalInstant(const PlanRequest& request) {
    Point targetBallPoint = CapturePlanner::getGoalInstant(request).pose.position();
    double a = *_targetSensitivity;
    std::optional<Point>& avgTargetBallPoint = _avgTargetBallPoints[request.shellID];
    if(!avgTargetBallPoint) {
        avgTargetBallPoint = targetBallPoint;
    } else {
        avgTargetBallPoint = a * targetBallPoint + (1-a) * *avgTargetBallPoint;
    }
    return RobotInstant{Pose{*avgTargetBallPoint, 0}, {}, RJ::Time(0s)};
}
//todo(Ethan) add ball as a dyn obs
std::optional<std::tuple<Trajectory, Pose, bool>> SettlePlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    Point futureBallPoint, futureBallVel;
    bool futureBallOutOfBounds = false;
    std::tie(futureBallPoint, futureBallVel, contactTime,
             futureBallOutOfBounds) = predictFutureBallState(ball, contactTime);

    ShapeSet static_obstacles_with_ball = static_obstacles;

    // Calculate the desired state of the robot at the point of ball contact
    // based on the command type
    Point targetFacePoint;
    double bufferDistBeforeContact = 0;
    if(futureBallVel.mag() < 0.2) {//todo(Ethan) no magic values
        targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
    } else {
        targetFacePoint = ball.pos - futureBallVel.normalized(10);
    }
    double contactSpeed = -futureBallVel.mag() * *_ballSpeedPercentForDampen;
    double timeUntilContact = RJ::Seconds{contactTime-RJ::now()}.count();
    double gapT = *_bufferTimeBeforeContact;
    if(!futureBallOutOfBounds && gapT > 0 && timeUntilContact > gapT) {
        auto [bufferBallPt, bufferBallVel, bufferTime, bufferOutOfBounds]
        = predictFutureBallState(ball, contactTime + RJ::Seconds{gapT});
        if(!bufferOutOfBounds) {
            bufferDistBeforeContact = (bufferBallPt - futureBallPoint).mag();
        }
    }
    static_obstacles_with_ball.add(std::make_shared<Circle>(ball.pos, Robot_Radius + Ball_Radius));

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
        debugThrow("Attack Angle in Settle too big");
        return std::nullopt;
    }

    // Course: approach the ball in line with the ball velocity
    std::optional<Trajectory> coursePath;
    Point courseTargetPoint = contactPoint - contactDir.normalized(bufferDistBeforeContact);
    Pose courseTargetPose{courseTargetPoint, 0};
    Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
    //if we aren't in the fine segment
    if(ball.pos.distTo(startInstant.pose.position()) > bufferDistBeforeContact + Robot_Radius + Ball_Radius) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles_with_ball, dynamic_obstacles);
        if(coursePath->empty()) {
            printf("attemptCapture: Course RRT Failed\n");
            return std::nullopt;
        }
    } else {
        printf("attemptCapture: in Fine, returning nullopt\n");
        //todo(Ethan) Really? (see below todo)
        return std::nullopt;
    }
    if(!coursePath) {
        printf("attemptCapture: contactVel=0, Course RRT Failed\n");
        return std::nullopt;
    }
    // Settle only uses Course path and then waits for the ball.
    // TODO: this might cause problems if the final robot vel != 0
    Trajectory out = std::move(*coursePath);
    PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
//  printf("attemptCapture: contactVel=0, using just Course\n");
    return std::make_tuple(std::move(out), contactPose, false);
}
}