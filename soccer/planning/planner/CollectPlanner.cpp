#include "CollectPlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
namespace Planning {
ConfigDouble* CollectPlanner::_goalPosChangeThreshold;
ConfigDouble* CollectPlanner::_goalVelChangeThreshold;
ConfigDouble* CollectPlanner::_bufferDistBeforeContact;
ConfigDouble* CollectPlanner::_bufferDistAfterContact;
ConfigDouble* CollectPlanner::_ballSpeedApproachDirectionCutoff;
ConfigDouble* CollectPlanner::_touchDeltaSpeed;

REGISTER_CONFIGURABLE(CollectPlanner);

void CollectPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg,
            "Capture/Collect/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg,
            "Capture/Collect/goalVelChangeThreshold");
    _bufferDistBeforeContact = new ConfigDouble(cfg,
            "Capture/Collect/bufferDistBeforeContact", 0);
    _bufferDistAfterContact = new ConfigDouble(cfg,
            "Capture/Collect/bufferDistAfterContact", 0);
    _ballSpeedApproachDirectionCutoff = new ConfigDouble(cfg,
             "Capture/Collect/ballSpeedApproachDirectionCutoff", 0);
    _touchDeltaSpeed = new ConfigDouble(cfg,
            "Capture/Collect/touchDeltaSpeed", 0);
}

using namespace Geometry2d;
//todo(Ethan) add ball as a dyn obs
std::optional<std::tuple<Trajectory, Pose, bool>> CollectPlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    Point futureBallPoint, futureBallVel;
    bool futureBallOutOfBounds = false;
    std::tie(futureBallPoint, futureBallVel, contactTime,
             futureBallOutOfBounds) = predictFutureBallState(ball, contactTime);

    ShapeSet static_obstacles_with_ball = static_obstacles;

    // Calculate the desired state of the robot at the point of ball contact
    // based on the command type
    Point targetFacePoint;
    double contactSpeed = 0;
    double bufferDistBeforeContact = 0;
    if(futureBallVel.mag() < *_ballSpeedApproachDirectionCutoff) {
        targetFacePoint = ball.pos + (ball.pos - startInstant.pose.position()).normalized(10);
    } else {
        targetFacePoint = ball.pos + futureBallVel.normalized(10);
    }
    contactSpeed = futureBallVel.mag() + *_touchDeltaSpeed;
    bufferDistBeforeContact = *_bufferDistBeforeContact;
    static_obstacles_with_ball.add(std::make_shared<Circle>(futureBallPoint, Robot_Radius + Ball_Radius));

    Point contactFaceDir;
    if(targetFacePoint.distTo(futureBallPoint) < 1e-9) {
        contactFaceDir = (futureBallPoint - startInstant.pose.position()).norm();
    } else {
        contactFaceDir = (targetFacePoint - futureBallPoint).norm();
    }
    //todo(change contactDir during linekick for a one-touch?
    Point contactDir = contactFaceDir;

    Point contactPoint = futureBallPoint - (Robot_MouthRadius + Ball_Radius) * contactFaceDir;
    Pose contactPose{contactPoint, contactFaceDir.angle()};
    contactSpeed = std::clamp(contactSpeed, -constraints.mot.maxSpeed, constraints.mot.maxSpeed);

    if(contactDir.angleBetween(contactFaceDir) > *_maxApproachAngle) {
        debugThrow("Attack Angle in Capture too big");
        return std::nullopt;
    }

    // Course: approach the ball in line with the ball velocity
    std::optional<Trajectory> coursePath;
    Point courseTargetPoint = contactPoint - contactDir.normalized(bufferDistBeforeContact);
    Pose courseTargetPose{courseTargetPoint, 0};
    Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
    Point botToContact = contactPoint-startInstant.pose.position();
    RJ::Time contactBallTime = RJ::Time::max();
    //if we aren't in the fine segment
    if(ball.pos.distTo(startInstant.pose.position()) > bufferDistBeforeContact + Robot_Radius + Ball_Radius) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles_with_ball, dynamic_obstacles);
        if(coursePath->empty()) {
            printf("attemptCapture: Course RRT Failed\n");
            return std::nullopt;
        }
        courseTargetInstant.stamp = coursePath->last().stamp;
        contactBallTime = coursePath->last().stamp;
    } else {
        printf("attemptCapture: in Fine, returning nullopt\n");
        //todo(Ethan) Really? (see below todo)
        return std::nullopt;
    }

    //Fine: constant velocity while contacting the ball
    RobotConstraints fineConstraints = constraints;
    fineConstraints.mot.maxAcceleration *= *_ballContactAccelPercent;
    fineConstraints.mot.maxSpeed = std::abs(contactSpeed);
    std::optional<Trajectory> finePathBeforeContact, finePathAfterContact;
    //Fine Path Before Contact
    Twist contactTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant contactInstant{contactPose, contactTwist, RJ::now()};
    bool isBeforeContact = coursePath || botToContact.mag() > Robot_MouthRadius + Ball_Radius;
    if(isBeforeContact) {
        RobotInstant instantBeforeFine = coursePath ? courseTargetInstant : startInstant;
        finePathBeforeContact = RRTTrajectory(instantBeforeFine, contactInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
        if(finePathBeforeContact->empty()) {
            if(coursePath) {
                Trajectory out = std::move(*coursePath);
                PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                printf("attemptCapture: Fine failed returning Course part\n");
                return std::make_tuple(std::move(out), contactPose, false);
            }
            printf("attemptCapture: both RRTs Failed\n");
            return std::nullopt;
        }
        contactInstant.stamp = finePathBeforeContact->last().stamp;
    }

    contactBallTime = contactInstant.stamp;
    //Fine Path After Contact
    if(isBeforeContact) {
        double stoppingDist = std::pow(contactSpeed, 2) / (2 * fineConstraints.mot.maxAcceleration);
        Point fineTargetPoint = contactPoint + contactDir.normalized(*_bufferDistAfterContact + stoppingDist);
        RobotInstant fineTargetInstant{Pose{fineTargetPoint, 0}, {}, RJ::Time(0s)};
        finePathAfterContact = RRTTrajectory(contactInstant, fineTargetInstant, fineConstraints.mot, static_obstacles, dynamic_obstacles);
        if(finePathAfterContact->empty()) {
            if(coursePath && finePathBeforeContact) {
                Trajectory out{std::move(*coursePath), std::move(*finePathBeforeContact)};
                PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                printf("attemptCapture: finePathAfter failed\n");
                return std::make_tuple(std::move(out), contactPose, false);
            } else if(coursePath) {
                Trajectory out = std::move(*coursePath);
                PlanAngles(out, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
                printf("attemptCapture: Fine before failed\n");
                return std::make_tuple(std::move(out), contactPose, false);
            }
            printf("attemptCapture: Course and After failed\n");
            return std::nullopt;
        }
    }

    //combine all the trajectory segments to build the output path
    Trajectory collectPath{{}};
    if(coursePath) {
        assert(finePathBeforeContact && finePathAfterContact);
        Trajectory finePath{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
        collectPath = Trajectory{std::move(*coursePath), std::move(finePath)};
        collectPath.setDebugText("Course");
    } else {
        printf("attemptCapture: too close to ball\n");
        //we're too close to the ball to trust anything. just reuse old path
        //todo(Ethan) REeally?
        return std::nullopt;
    }
    assert(!collectPath.empty());
    bool successfulCapture = contactBallTime - 1e-6s < contactTime;
    PlanAngles(collectPath, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
    return std::make_tuple(std::move(collectPath), contactPose, successfulCapture);
}
}