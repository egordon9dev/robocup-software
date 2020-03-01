#include "CollectPlanner.hpp"
namespace Planning {
ConfigDouble* CollectPlanner::_goalPosChangeThreshold;
ConfigDouble* CollectPlanner::_goalVelChangeThreshold;

REGISTER_CONFIGURABLE(CollectPlanner);

void CollectPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg, "Capture/Collect/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg, "Capture/Collect/goalVelChangeThreshold");
}

using namespace Geometry2d;
RobotInstant CollectPlanner::getGoalInstant(const PlanRequest& request) {
    //TODO do this in the parent!!!
    auto bruteForceResult = bruteForceCapture(request);
    Trajectory path{{}};
    if(bruteForceResult) {
        std::tie(_contactTime, path) = std::move(*bruteForceResult);
    }
    if(path.empty()) {
        Point ballPoint = request.context->state.ball.pos;
        Point startPoint = request.start.pose.position();
        return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
    }
    return path.last();
}

}