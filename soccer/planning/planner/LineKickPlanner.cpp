#include "LineKickPlanner.hpp"
namespace Planning {
ConfigDouble* LineKickPlanner::_goalPosChangeThreshold;
ConfigDouble* LineKickPlanner::_goalVelChangeThreshold;

REGISTER_CONFIGURABLE(LineKickPlanner);

void LineKickPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg, "LineKickPlanner/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg, "LineKickPlanner/goalVelChangeThreshold");
}

using namespace Geometry2d;
RobotInstant LineKickPlanner::getGoalInstant(const PlanRequest& request) {
    //TODO do this in the parent!!!
    auto bruteForceResult = bruteForceCapture(request);
    if(bruteForceResult) {
        return RobotInstant{std::get<1>(*bruteForceResult), {}, RJ::Time(0s)};
    }
    Point ballPoint = request.context->state.ball.pos;
    Point startPoint = request.start.pose.position();
    return RobotInstant{Pose{ballPoint,startPoint.angleTo(ballPoint)}, {}, RJ::now()};
}
}