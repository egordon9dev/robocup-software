#include "LineKickPlanner.hpp"
#include "planning/trajectory/RRTUtil.hpp"
namespace Planning {
ConfigDouble* LineKickPlanner::_goalPosChangeThreshold;
ConfigDouble* LineKickPlanner::_goalVelChangeThreshold;
ConfigDouble* LineKickPlanner::_bufferDistBeforeContact;
ConfigDouble* LineKickPlanner::_bufferDistAfterContact;

REGISTER_CONFIGURABLE(LineKickPlanner);

void LineKickPlanner::createConfiguration(Configuration* cfg) {
    _goalPosChangeThreshold = new ConfigDouble(cfg,
            "LineKickPlanner/goalPosChangeThreshold");
    _goalVelChangeThreshold = new ConfigDouble(cfg,
            "LineKickPlanner/goalVelChangeThreshold");
    _bufferDistBeforeContact = new ConfigDouble(cfg,
            "LineKickPlanner/bufferDistBeforeContact");
    _bufferDistAfterContact = new ConfigDouble(cfg,
            "LineKickPlanner/bufferDistAfterContact");
}

using namespace Geometry2d;
//todo(Ethan) add ball as a dyn obs
std::optional<std::tuple<Trajectory, Pose, bool>> LineKickPlanner::attemptCapture(const PlanRequest& request, RJ::Time contactTime) const {
    const Ball& ball = request.context->state.ball;
    const ShapeSet& static_obstacles = request.static_obstacles;
    const std::vector<DynamicObstacle>& dynamic_obstacles = request.dynamic_obstacles;
    RobotInstant startInstant = request.start;
    const RobotConstraints& constraints = request.constraints;

    Point futureBallPoint, futureBallVel;
    bool fakeFutureBall = false;
    std::tie(futureBallPoint, futureBallVel,contactTime,
             fakeFutureBall) = predictFutureBallState(ball, contactTime);

    // Calculate the desired state of the robot at the point of ball contact
    // based on the command type
    Point targetFacePoint;
    double contactSpeed = 0;

    auto lineKickCommand = std::get<LineKickCommand>(request.motionCommand);
    targetFacePoint = lineKickCommand.target;
    contactSpeed = lineKickApproachSpeed;

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
    Point courseTargetPoint = contactPoint - contactDir.normalized(*_bufferDistBeforeContact);
    Pose courseTargetPose{courseTargetPoint, 0};
    Twist courseTargetTwist{contactDir.normalized(contactSpeed), 0};
    RobotInstant courseTargetInstant{courseTargetPose, courseTargetTwist, RJ::now()};
    Point botToContact = contactPoint-startInstant.pose.position();
    RJ::Time contactBallTime = RJ::Time::max();
    //if we aren't in the fine segment
    if(ball.pos.distTo(startInstant.pose.position()) > *_bufferDistBeforeContact + Robot_Radius + Ball_Radius) {
        coursePath = RRTTrajectory(startInstant, courseTargetInstant, constraints.mot, static_obstacles, dynamic_obstacles);
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
    Trajectory linekickPath{{}};
    if(coursePath) {
        // this is the Main Success Scenario for a Linekick Command
//        printf("attemptCapture: Success, returning full path\n");
        assert(finePathBeforeContact && finePathAfterContact);
        Trajectory finePath{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
        linekickPath = Trajectory{std::move(*coursePath), std::move(finePath)};
        linekickPath.setDebugText("Course");
//        } else if (finePathBeforeContact) {
//            assert(finePathAfterContact);
//            linekickPath = Trajectory{std::move(*finePathBeforeContact), std::move(*finePathAfterContact)};
//            linekickPath.setDebugText("Fine");
    } else {
        printf("attemptCapture: too close to ball\n");
        //we're too close to the ball to trust anything. just reuse old path
        //todo(Ethan) REeally?
        return std::nullopt;
    }
    assert(!linekickPath.empty());
    bool successfulCapture = contactBallTime - 1e-6s < contactTime;
    PlanAngles(linekickPath, startInstant, AngleFns::faceAngle(contactFaceDir.angle()), constraints.rot);
    return std::make_tuple(std::move(linekickPath), contactPose, successfulCapture);
}
}