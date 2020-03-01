#pragma once
#include "CapturePlanner.hpp"
namespace Planning {
class SettlePlanner: public CapturePlanner {
public:
    SettlePlanner(): avgTargetPoints(Num_Shells, std::nullopt) {}

    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<SettleCommand>(command);
    }

    double goalPosChangeThreshold() const override {
        return *_goalPosChangeThreshold;
    }
    double goalVelChangeThreshold() const override {
        return *_goalVelChangeThreshold;
    }

protected:
    RobotInstant getGoalInstant(const PlanRequest& request) override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;

    std::vector<std::optional<Geometry2d::Point>> avgTargetPoints;
};
}