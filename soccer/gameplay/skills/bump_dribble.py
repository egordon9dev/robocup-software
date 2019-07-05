import single_robot_composite_behavior
import behavior
import enum
import skills.move
import skills.bump
import main
import role_assignment
import time

## Behavior that dribbles with the ball as far as possible
# a legal version of dribble
class BumpDribble(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        move = 1

        bump = 2

    STRIKE_DISTANCE = 0.15


    def __init__(self, pos=None):
        super().__init__(continuous=False)

        for s in BumpDribble.State :
            self.add_state(s, behavior.Behavior.State.running)

        self.threshold = 0.05
        self.pos = pos
        self.timer = time.time()

        self.add_transition(behavior.Behavior.State.start,
                            BumpDribble.State.move, lambda: True,
                            'immediately')
        self.add_transition(BumpDribble.State.move,
                            BumpDribble.State.bump,
                            lambda: self.is_far_enough(),
                            'bump')
        self.add_transition(BumpDribble.State.bump,
                            BumpDribble.State.move,
                            lambda: time.time() - self.timer > 1.0,#self.subbehavior_with_name('bump').State == behavior.Behavior.State.completed,
                            're-target')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.pos != None and (self.robot.pos - self.pos).mag() < self.threshold,
            'target pos reached')
        self.add_transition(
            behavior.Behavior.State.completed, BumpDribble.State.move,
            lambda: self.pos != None and (self.robot.pos - self.pos).mag() > self.threshold,
            'away from target')

    ## the position to move to (a robocup.Point object)
    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    ## how close (in meters) the robot has to be to the target position for it be complete
    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value

    #def on_enter_move(self):
    #    ball_pos = self.robot.pos
    #    self.add_subbehavior(skills.move.Move(ball_pos),'move',required = False, priority = 4)

    #def on_exit_move(self):
    #    self.remove_subbehavior('move')

    def on_enter_bump(self):
        self.timer = time.time()
        bump = skills.bump.Bump()
        bump.target = self.pos
        self.add_subbehavior(bump, 'bump', required = False, priority = 4)

    def on_exit_bump(self):
        self.remove_subbehavior('bump')

    def is_far_enough(self):
        return (main.ball().pos - self.robot.pos).mag() > BumpDribble.STRIKE_DISTANCE 

    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs