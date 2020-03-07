#pragma once
#include "CapturePlanner.hpp"
namespace Planning {
class SettlePlanner: public CapturePlanner {
public:
    SettlePlanner(): _avgTargetBallPoints(Num_Shells, std::nullopt) {}

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

    bool veeredOffPath(const PlanRequest& req) const override { return false; }

    Trajectory fullReplan(PlanRequest&& req, RobotInstant goalInstant, AngleFunction angleF) override {
        const Trajectory& prevTraj = req.prevTrajectory;
        if(!prevTraj.empty()) {
            RJ::Time startT = req.start.stamp;
            std::optional<RobotInstant> optStart = prevTraj.evaluate(startT);
            if(optStart) {
                req.start = *optStart;
            }
        }
        return CapturePlanner::fullReplan(std::move(req), goalInstant, angleF);
    }

protected:
    RobotInstant getGoalInstant(const PlanRequest& request) override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;

    std::vector<std::optional<Geometry2d::Point>> _avgTargetBallPoints;
};
}