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
    auto bruteForceResult = bruteForceCapture(request);
    Trajectory path{{}};
    Pose contactPose;
    if(bruteForceResult) {
        //TODO: this part should probably be done in the parent class
        std::tie(path, contactPose) = std::move(*bruteForceResult);
    }
    if(path.empty() || 1) {
        Point ballPoint = request.context->state.ball.pos;
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