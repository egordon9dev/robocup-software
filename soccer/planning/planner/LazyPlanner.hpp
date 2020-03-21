#pragma once

#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include <vector>

namespace Planning {
/**todo(Ethan) fix description
 * @brief abstract Planner that conducts re-planning.
 * it's lazy because it only makes a new plan when the old plan is invalid, but
 * occasionally it will check for a better path even if the old path is valid.
 *
 * Derived classes just need to implement getGoalInstant()
 * other methods are provided with default implementations but can be overriden
 * no need to override plan()!
 *
 * all command types are converted to PathTargetCommands with the pathGoal
 * given by getGoalInstant(request)
 *
 * NOTE: call updatePrevTime(shell_id) whenever a new path is returned
 */
class LazyPlanner: public Planner {
public:
    LazyPlanner(std::string name,
                ConfigDouble* const& goalPosChange,
                ConfigDouble* const& goalVelChange,
                ConfigDouble* const& partialReplanLeadTime = defaultPartialReplanLeadTime):
            Planner(name),
            _goalPosChangeThreshold(goalPosChange),
            _goalVelChangeThreshold(goalVelChange),
            _partialReplanLeadTime(partialReplanLeadTime){}
    virtual ~LazyPlanner() = default;

    Trajectory plan(PlanRequest&& request) override;

    static void createConfiguration(Configuration* cfg);

protected:
    /**
     * create a new trajectory and discard the previous trajectory
     * @param request
     * @param angleFunction
     * @return trajectory
     */
    virtual Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant) = 0;

    /**
     * use a small piece of the previous trajectory replan from that point forward
     * @param request
     * @param angleFunction
     * @return trajectory
     */
    virtual Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant) = 0;

    /**
     * check for a better path even if the current path is valid.
     * @param request
     * @param angleFunction
     * @return better trajectory if one is found; otherwise reuse old path
     */
    virtual Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant);

    /**
     * check if the goal changed enough to cause a replan
     * @param prevGoal
     * @param goal
     * @return true if the goal changed, false otherwise
     */
    virtual bool goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const;

    /**
     * get the current goal instant given a plan request
     * @param request
     * @return goal instant (time stamp is undefined)
     */
    virtual RobotInstant getGoalInstant(const PlanRequest& request) = 0;

    virtual Trajectory partialPath(const Trajectory& prevTrajectory) const {
        //todo(Ethan) cut out old parts of the path?
        return prevTrajectory.subTrajectory(0s, (RJ::now() - prevTrajectory.begin_time()) + RJ::Seconds{*_partialReplanLeadTime});
    }

    /**
     * reuse the previous trajectory
     * @param request
     * @return trajectory
     */
    virtual Trajectory reuse(PlanRequest&& request);

    virtual bool veeredOffPath(const PlanRequest& request) const;

private:
    static ConfigDouble* defaultPartialReplanLeadTime;
    ConfigDouble* const& _partialReplanLeadTime;
    ConfigDouble* const& _goalPosChangeThreshold;
    ConfigDouble* const& _goalVelChangeThreshold;
};

}