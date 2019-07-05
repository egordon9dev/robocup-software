import single_robot_composite_behavior
import behavior
import enum
import skills.dribble
import skills.bump_dribble
import main
import role_assignment
import time

## Behavior that combines both benifits of bump dribble and regular dribble
# a cautious dribble
class CautiousDribble(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        dribbler = 1

        bump = 2


    def __init__(self, pos=None):
        super().__init__(continuous=False)

        for s in CautiousDribble.State :
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
        					CautiousDribble.State.bump,
        					lambda: True,
        					'immediately')

        self.add_transition(CautiousDribble.State.bump,
        					CautiousDribble.State.dribbler,
        					lambda: self.can_be_stolen() or self.final_approach(),
        					'dribble')

        self.add_transition(CautiousDribble.State.dribbler,
        					CautiousDribble.State.bump,
        					lambda: not self.can_be_stolen() or self.reached_limit(),
        					'bump')

        self.threshold = 0.05
        self.max_dribble_dist = 0.9
        self.danger_dist = 0.5
        self.final_approach_dist = 0.3
        self.pos = pos
        self.start_dribbler_pos = None

    def can_be_stolen(self):
    	check = False
    	for r in main.their_robots() :
    		check = check or (r.pos - main.ball().pos).mag() < self.danger_dist
    	return check

    def final_approach(self):
    	return (self.robot.pos - self.pos).mag()

    def reached_limit(self):
    	return (self.start_dribbler_pos - self.robot.pos).mag() < self.max_dribble_dist

    def on_enter_dribbler(self):
    	self.start_dribbler_pos = self.robot.pos
    	dribble = skills.dribble.Dribble(self.pos)
    	self.add_subbehavior(dribble, 'dribble', required = False, priority = 4)

    def on_exit_dribbler(self):
    	self.remove_all_subbehaviors()

    def on_enter_bump(self):
    	bump = skills.bump_dribble.BumpDribble(self.pos)
    	self.add_subbehavior(bump, 'bump', required = False, priority = 4)

    def on_exit_bump(self):
    	self.remove_all_subbehaviors()

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs