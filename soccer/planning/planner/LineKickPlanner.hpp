#pragma once

#include "CapturePlanner.hpp"
namespace Planning{
class LineKickPlanner: public CapturePlanner {
public:
    LineKickPlanner(): CapturePlanner("LineKickPlanner", _goalPosChangeThreshold,
                                      _goalVelChangeThreshold) {}
    ~LineKickPlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<LineKickCommand>(command);
    }

protected:
    std::optional<std::tuple<Trajectory, Geometry2d::Pose, bool>> attemptCapture(const PlanRequest& request, RJ::Time contactTime) const override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;

    // buffer distance for when we contact the ball. between these buffers, the
    // robot will move at a constant velocity
    static ConfigDouble* _bufferDistBeforeContact;
    static ConfigDouble* _bufferDistAfterContact;
};
}