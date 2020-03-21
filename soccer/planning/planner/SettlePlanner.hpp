#pragma once
#include "CapturePlanner.hpp"
namespace Planning {
class SettlePlanner: public CapturePlanner {
public:
    SettlePlanner(): CapturePlanner("LineKickPlanner", _goalPosChangeThreshold,
      _goalVelChangeThreshold),_avgTargetBallPoints(Num_Shells, std::nullopt) {}
    ~SettlePlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<SettleCommand>(command);
    }

    bool veeredOffPath(const PlanRequest& req) const override { return false; }

    Trajectory fullReplan(PlanRequest&& req, RobotInstant goalInstant) override {
        const Trajectory& prevTraj = req.prevTrajectory;
        if(!prevTraj.empty()) {
            RJ::Time startT = req.start.stamp;
            std::optional<RobotInstant> optStart = prevTraj.evaluate(startT);
            if(optStart) {
                req.start = *optStart;
            }
        }
        return CapturePlanner::fullReplan(std::move(req), goalInstant);
    }

protected:
    std::optional<std::tuple<Trajectory, Geometry2d::Pose, bool>> attemptCapture(const PlanRequest& request, RJ::Time contactTime) const override;
    RobotInstant getGoalInstant(const PlanRequest& request) override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;

    // duration of the buffer before contacting the ball where the robot will
    // move at a constant velocity
    static ConfigDouble* _bufferTimeBeforeContact;

    // percent of the ball vel we match during dampen (when we catch a ball
    // moving fast toward us)
    static ConfigDouble* _ballSpeedPercentForDampen; // %

    // Gain on the averaging function to smooth the ball velocity to for any
    // motion commands This is due to the high flucations in the ball velocity
    // frame to frame a*newPoint + (1-a)*oldPoint The lower the number, the less
    // noise affects the system, but the slower it responds to changes The
    // higher the number, the more noise affects the system, but the faster it
    // responds to changes
    static ConfigDouble* _targetSensitivity;

    std::vector<std::optional<Geometry2d::Point>> _avgTargetBallPoints;
};
}