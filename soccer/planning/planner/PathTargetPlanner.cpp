#include "planning/planner/PathTargetPlanner.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "planning/trajectory/RRTUtil.hpp"

namespace Planning {

ConfigDouble* PathTargetPlanner::_goalPosChangeThreshold;
ConfigDouble* PathTargetPlanner::_goalVelChangeThreshold;

REGISTER_CONFIGURABLE(PathTargetPlanner);

void PathTargetPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg, "PathTargetPlanner/goalChangeThreshold", 0);
    _goalVelChangeThreshold = new ConfigDouble(cfg, "PathTargetPlanner/velChangeThreshold", 0);
}

RobotInstant PathTargetPlanner::getGoalInstant(const PlanRequest& request) {
    auto& command = std::get<PathTargetCommand>(request.motionCommand);
    RobotInstant goalInstant = command.pathGoal;
    Geometry2d::Point& goalPoint = goalInstant.pose.position();
    std::optional<Geometry2d::Point> prevGoal;
    if (!request.prevTrajectory.empty()) {
        prevGoal = request.prevTrajectory.last().pose.position();
    }
    goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(goalPoint,
                                                               prevGoal,
                                                               request.static_obstacles);
    return goalInstant;
}

Trajectory PathTargetPlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, std::move(request.static_obstacles));
    const RJ::Seconds timeIntoTrajectory =
            RJ::now() - prevTrajectory.begin_time();
    Trajectory preTrajectory = partialPath(prevTrajectory);
    Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
    PlanAngles(postTrajectory, preTrajectory.last(), AngleFns::zero, request.constraints.rot);
    if (!postTrajectory.empty()) {
        Trajectory comboPath{std::move(preTrajectory),std::move(postTrajectory)};
        if (prevTrajectory.duration() - timeIntoTrajectory > comboPath.duration()) {
            std::cout << "Found A better Path!!!!" << std::endl;
            return std::move(comboPath);
        }
    }
    return reuse(std::move(request));
}

Trajectory PathTargetPlanner::partialReplan(PlanRequest&& request, RobotInstant goalInstant) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    std::vector<Geometry2d::Point> biasWaypoints;
    for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
         (*it).stamp < prevTrajectory.end_time(); ++it) {
        biasWaypoints.push_back((*it).pose.position());
    }
    Trajectory preTrajectory = partialPath(prevTrajectory);
    Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles, biasWaypoints);
    if (postTrajectory.empty()) {
        return fullReplan(std::move(request), goalInstant);
    }
    Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                      std::move(postTrajectory));
    std::optional<double> angle_override =
            request.context->robot_intents[request.shellID].angle_override;
    if(angle_override) {
        PlanAngles(comboPath, comboPath.first(),
           AngleFns::faceAngle(*angle_override), request.constraints.rot);
    }
    return std::move(comboPath);
}

Trajectory PathTargetPlanner::fullReplan(PlanRequest&& request, RobotInstant goalInstant) {
    Trajectory path = RRTTrajectory(request.start, goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
    if(path.empty()) {
        return reuse(std::move(request));
    }
    std::optional<double> angle_override =
            request.context->robot_intents[request.shellID].angle_override;
    if(angle_override) {
        PlanAngles(path, path.first(),
           AngleFns::faceAngle(*angle_override), request.constraints.rot);
    }
    return std::move(path);
}

}