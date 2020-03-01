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
    if(bruteForceResult) {
        //TODO: this part should probably be done in the parent class
        std::tie(_contactTime, path) = std::move(*bruteForceResult);
    }
    if(path.empty()) {
        Point ballPoint = request.context->state.ball.pos;
        Point startPoint = request.start.pose.position();
        return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
    }
    double a = *_settleTargetSensitivity;
    Point targetPoint = path.last().pose.position();
    std::optional<Point>& avgTargetPoint = avgTargetPoints[request.shellID];
    if(!avgTargetPoint) {
        avgTargetPoint = targetPoint;
    } else {
        avgTargetPoint = a * targetPoint + (1-a) * *avgTargetPoint;
    }
    RobotInstant targetInstant = path.last();
    targetInstant.pose.position() = *avgTargetPoint;
    return targetInstant;

}
}