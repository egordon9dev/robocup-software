#include "SettlePlanner.hpp"
namespace Planning {
ConfigDouble* SettlePlanner::_goalPosChangeThreshold;
ConfigDouble* SettlePlanner::_goalVelChangeThreshold;

REGISTER_CONFIGURABLE(SettlePlanner);

void SettlePlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg, "Capture/Settle/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg, "Capture/Settle/goalVelChangeThreshold");
}
using namespace Geometry2d;
RobotInstant SettlePlanner::getGoalInstant(const PlanRequest& request) {
    const Ball& ball = request.context->state.ball;
    auto bruteForceResult = bruteForceCapture(request);
    Trajectory path{{}};
    Pose contactPose;
    if(bruteForceResult) {
        std::tie(path, contactPose) = std::move(*bruteForceResult);
    }
    if(path.empty()) {
        std::optional<Point> prevTarget = _avgTargetBallPoints[request.shellID];
        Point ballPoint = prevTarget ? *prevTarget : ball.pos;
        Point startPoint = request.start.pose.position();
        return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
    }
    Point targetBallPoint = contactPose.position() + Point::direction(contactPose.heading()).normalized(Robot_MouthRadius + Ball_Radius);
    double a = *_settleTargetSensitivity;
    std::optional<Point>& avgTargetBallPoint = _avgTargetBallPoints[request.shellID];
    if(!avgTargetBallPoint) {
        avgTargetBallPoint = targetBallPoint;
    } else {
        avgTargetBallPoint = a * targetBallPoint + (1-a) * *avgTargetBallPoint;
    }
    return RobotInstant{Pose{*avgTargetBallPoint, 0}, {}, RJ::Time(0s)};
}
}