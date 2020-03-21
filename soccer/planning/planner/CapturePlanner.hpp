#pragma once

#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
/*
 * Abstract class for shared logic between Collect, Settle, and LineKick
 * Plans a trajectory to the ball. After we reach the ball, it's up to Gameplay
 * to figure out what to do next (i.e. pass, shoot, pivot, etc.)
 */
namespace Planning {
class CapturePlanner: public LazyPlanner {
public:
    CapturePlanner(std::string name,
                   ConfigDouble* const& goalPosChange,
                   ConfigDouble* const& goalVelChange):
            LazyPlanner(name, goalPosChange, goalVelChange) {}
    ~CapturePlanner() override = default;

    static void createConfiguration(Configuration* cfg);

protected:
    /**
     * The Goal is the location of the ball at contact.
     * @param request
     * @return RobotInstant with the ball Point. other fields are undefined
     */
    RobotInstant getGoalInstant(const PlanRequest& request) override;
    Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant) override;
    Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant) override;
    Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant) override;

    Geometry2d::Point projectPointIntoField(Geometry2d::Point targetPoint, const Geometry2d::Rect& fieldRect, Geometry2d::Point ballPoint) const;

    std::tuple<Geometry2d::Point, Geometry2d::Point, RJ::Time, bool>
    predictFutureBallState(const Ball& ball, RJ::Time contactTime) const;

    /**
     * attempt to build a trajectory to capture the ball
     * @param request
     * @param contactTime
     * @return a tuple of (path, contactPose, success) where path is the
     * output trajectory, and contactPose is the Pose of the robot at the point
     * of contact with the ball
     */
    virtual std::optional<std::tuple<Trajectory, Geometry2d::Pose, bool>> attemptCapture(const PlanRequest& request, RJ::Time contactTime) const = 0;

    /**
     * find a trajectory from the given request to capture the ball.
     * searches for the trajectory with minimum contact time
     * @param request
     * @return tuple of (bestPath, pose) where bestPath is the best trajectory
     * found and pose is the Pose of the robot at contact
     */
    std::optional<std::tuple<Trajectory, Geometry2d::Pose>> bruteForceCapture(const PlanRequest& request) const;

    // When trying to reach the targetFacePoint it might be more efficient
    // to approach the ball at an angle rather than always staying orthogonal
    // this is the maximum angle away from orthogonal we will go
    static ConfigDouble* _maxApproachAngle; // radians

    //the maximum acceleration while stopping with the ball.
    // (as a percent of MotionConstraints.maxAcceleration)
    static ConfigDouble* _ballContactAccelPercent; // %

    static constexpr double lineKickApproachSpeed = 0.25;
};
}