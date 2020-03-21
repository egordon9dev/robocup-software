#pragma once
#include "LazyPlanner.hpp"
#include <vector>
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

class PathTargetPlanner: public LazyPlanner {
public:
    PathTargetPlanner(): LazyPlanner("PathTargetPlanner", _goalPosChangeThreshold,
                                     _goalVelChangeThreshold) {}
    ~PathTargetPlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    //todo(Ethan) write WorldVelPlanner
    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<PathTargetCommand>(command);
    }

protected:
    Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant) override;
    Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant) override;
    Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant) override;

    RobotInstant getGoalInstant(const PlanRequest& request) override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
};
} // namespace Planning
