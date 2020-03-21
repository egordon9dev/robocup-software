#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <vector>

namespace Planning {
    ConfigDouble* LazyPlanner::defaultPartialReplanLeadTime;

    REGISTER_CONFIGURABLE(LazyPlanner);

    void LazyPlanner::createConfiguration(Configuration* cfg) {
        defaultPartialReplanLeadTime = new ConfigDouble(cfg, "LazyPlanner/partialReplanLeadTime", 0.0);
    }
    using namespace Geometry2d;
    Trajectory LazyPlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant) {
        PlanRequest requestCopy = request;
        Trajectory newTrajectory = partialReplan(std::move(request), goalInstant);
        if(newTrajectory.end_time() < requestCopy.prevTrajectory.end_time()) {
            return std::move(newTrajectory);
        }
        return reuse(std::move(requestCopy));
    }

    bool LazyPlanner::veeredOffPath(const PlanRequest& request) const {
        const auto &currentInstant = request.start;
        const MotionConstraints &motionConstraints = request.constraints.mot;
        const Trajectory& prevTrajectory = request.prevTrajectory;
        if (prevTrajectory.empty()) return false;
        RJ::Seconds timeIntoPath =
                (RJ::now() - prevTrajectory.begin_time()) + RJ::Seconds(1) / 60;
        std::optional<RobotInstant> optTarget = prevTrajectory.evaluate(timeIntoPath);
        // If we went off the end of the path, use the end for calculations.
        RobotInstant target = optTarget ? *optTarget : prevTrajectory.last();
        // invalidate path if current position is more than the replanThreshold away
        // from where it's supposed to be right now
        float pathError = (target.pose.position() - currentInstant.pose.position()).mag();
        float replanThreshold = *motionConstraints._replan_threshold;
        if (replanThreshold != 0 && pathError > replanThreshold) {
            return true;
        }
        return false;
    }

    // todo(Ethan) use pathGoal heading and angular velocity?
    /**
     * Implement lazy planning. depending on the situation we full replan, partial
     * replan, or reuse the previous path.
     * @tparam CommandType
     * @param request
     * @return trajectory if one can be found, otherwise nullopt
     */
    Trajectory LazyPlanner::plan(PlanRequest &&request) {
        const Trajectory& prevTrajectory = request.prevTrajectory;

        RobotInstant goalInstant = getGoalInstant(request);
        Point& goalPoint = goalInstant.pose.position();
        // Simple case: no path
        //todo(Ethan) maybe delete this handle it in RRTTrajectory() ?
        if (request.start.pose.position().distTo(goalPoint) < 1e-6) {
            std::list<RobotInstant> instants;
            instants.emplace_back(request.start.pose, Twist(), RJ::now());
            Trajectory result{std::move(instants)};
            result.setDebugText("RRT Basic");
            return std::move(result);
        }
        if (prevTrajectory.empty() || veeredOffPath(request)) {
            return fullReplan(std::move(request), goalInstant);
        }
        RJ::Seconds timeIntoTrajectory =
                std::clamp(RJ::Seconds{RJ::now() - prevTrajectory.begin_time()}, RJ::Seconds{0s}, prevTrajectory.duration());
        const RJ::Seconds timeRemaining =
                prevTrajectory.duration() - timeIntoTrajectory;

        RJ::Seconds invalidTime;
        //note: the dynamic check is expensive, so we shortcut it sometimes
        bool shouldPartialReplan = prevTrajectory.hit(request.static_obstacles, timeIntoTrajectory, &invalidTime)
                                   || prevTrajectory.intersects(request.dynamic_obstacles, RJ::now(), nullptr, &invalidTime);
        if(!shouldPartialReplan && goalChanged(prevTrajectory.last(), goalInstant)) {
            shouldPartialReplan = true;
            invalidTime = prevTrajectory.duration();
        }
        if (shouldPartialReplan) {
            if (invalidTime - timeIntoTrajectory < RJ::Seconds{*_partialReplanLeadTime} * 2) {
                return fullReplan(std::move(request), goalInstant);
            }
            return partialReplan(std::move(request), goalInstant);
        }
        // make fine corrections when we are realy close to the target
        // because the old target might be a bit off
        if(request.start.pose.position().distTo(goalPoint) < Robot_Radius) {
            std::optional<RobotInstant> nowInstant = prevTrajectory.evaluate(RJ::now());
            if (nowInstant) {
                request.start = *nowInstant;
                return fullReplan(std::move(request), goalInstant);
            }
        }
        //todo(Ethan) fix this!!!!
        if (RJ::now() - request.prevTrajectory.timeCreated() > 0.2s && timeRemaining > RJ::Seconds{*_partialReplanLeadTime} * 2) {
            return checkBetter(PlanRequest{request}, goalInstant);
        }
        return reuse(std::move(request));
    }
    Trajectory LazyPlanner::reuse(PlanRequest&& request) {
        Trajectory& prevTrajectory = request.prevTrajectory;
        if(prevTrajectory.empty()) {
            Trajectory out{{request.start}};
            out.setDebugText("Empty");
            return std::move(out);
        }
        RJ::Seconds timeElapsed = RJ::now() - prevTrajectory.begin_time();
        //todo(Ethan) does this hurt performance???
        if(timeElapsed < prevTrajectory.duration()) {
            prevTrajectory.trimFront(timeElapsed);
            Trajectory out = std::move(prevTrajectory);
            out.setDebugText("Reuse");
            return std::move(out);
        }
        Trajectory out{{prevTrajectory.last()}};
        out.setDebugText("Reusing Past End");
        return std::move(out);
    }
    bool LazyPlanner::goalChanged(const RobotInstant &prevGoal,
                                        const RobotInstant &goal) const {
        double goalPosDiff = (prevGoal.pose.position() -
                              goal.pose.position()).mag();
        double goalVelDiff = (prevGoal.velocity.linear() -
                              goal.velocity.linear()).mag();
        return goalPosDiff > *_goalPosChangeThreshold
               || goalVelDiff > *_goalVelChangeThreshold;
    }
}