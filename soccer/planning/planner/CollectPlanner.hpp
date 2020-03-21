#pragma once
#include "CapturePlanner.hpp"
namespace Planning {
class CollectPlanner: public CapturePlanner {
public:
    CollectPlanner(): CapturePlanner("CollectPlanner", _goalPosChangeThreshold,
            _goalVelChangeThreshold) {}
    ~CollectPlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    bool isApplicable(const MotionCommand& command) const override {
        return std::holds_alternative<CollectCommand>(command);
    }

protected:
    std::optional<std::tuple<Trajectory, Geometry2d::Pose, bool>> attemptCapture(const PlanRequest& request, RJ::Time contactTime) const override;

private:
    static ConfigDouble* _goalPosChangeThreshold;
    static ConfigDouble* _goalVelChangeThreshold;


    // At what speed should we be when we touch the ball (Well, the difference
    // in speed between the ball and robot) Should be as low as possible where
    // we still are able to touch the ball and control it If we are slamming
    // into the ball decrease this number If we aren't even touching it to the
    // dribbler, increase this number
    static ConfigDouble* _touchDeltaSpeed; // m/s

    // buffer distance for when we contact the ball. between these buffers, the
    // robot will move at a constant velocity
    static ConfigDouble* _bufferDistBeforeContact;
    static ConfigDouble* _bufferDistAfterContact;

    // Controls at which ball speed we should try to go directly to the ball
    // or to move behind it and in the same direction as it
    //
    // Low number indicates that it will always try to choose a point for the
    // robot behind the velocity vector
    //
    // High number indicates that it will always try to choose a point nearest
    // to the current robot position
    static ConfigDouble* _ballSpeedApproachDirectionCutoff;  // m/s
};
}