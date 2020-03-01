#pragma once

#include "LazyPlanner.hpp"
#include <vector>
#include "planning/trajectory/VelocityProfiling.hpp"

namespace Planning {

class PathTargetPlanner: public LazyPlanner {
public:
    PathTargetPlanner() = default;
    virtual ~PathTargetPlanner() = default;

    static void createConfiguration(Configuration* cfg);

    std::string name() const override { return "PathTargetPlanner"; }

    //todo(Ethan) write WorldVelPlanner
    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<PathTargetCommand>(command);
    }

    double goalPosChangeThreshold() const override {
        return *_goalPosChangeThreshold;
    }
    double goalVelChangeThreshold() const override {
        return *_goalVelChangeThreshold;
    }

protected:
    Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) override;
    Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) override;
    Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) override;

    RobotInstant getGoalInstant(const PlanRequest& request) override {
        return std::get<PathTargetCommand>(request.motionCommand).pathGoal;
    }

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;
};
} // namespace Planning
