package frc.robot.route;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import org.littletonrobotics.junction.Logger;

// CoarseRouteProvider backed by BLine-Lib's FollowPath (evaluation only).
//
// WARNING: BLine is a geometric path follower, not a dynamic pathfinder. A
// single-waypoint request (as built here) drives a direct, straight-line route from
// wherever the robot currently is to the target pose and performs NO obstacle
// avoidance - unlike PathPlannerCoarseRouteProvider's navgrid-aware
// AutoBuilder.pathfindToPose. Only route through this provider where the straight-line
// path between the robot's current pose and the target is known to be physically clear.
public class BLineCoarseRouteProvider implements CoarseRouteProvider {

  private final FollowPath.Builder routeBuilder;

  public BLineCoarseRouteProvider(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator) {
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        Constants.BLineAuto.DEFAULT_MAX_VELOCITY_MPS,
        Constants.BLineAuto.DEFAULT_MAX_ACCELERATION_MPS2,
        Constants.BLineAuto.DEFAULT_MAX_VELOCITY_DEG_PER_SEC,
        Constants.BLineAuto.DEFAULT_MAX_ACCELERATION_DEG_PER_SEC2,
        Constants.BLineAuto.DEFAULT_END_TRANSLATION_TOLERANCE_METERS,
        Constants.BLineAuto.DEFAULT_END_ROTATION_TOLERANCE_DEG,
        Constants.BLineAuto.DEFAULT_INTERMEDIATE_HANDOFF_RADIUS_METERS));

    // No withShouldFlip/withDefaultShouldFlip/withShouldMirror: the Builder's flip/mirror
    // suppliers default to "never flip," and RouteRequest.targetPose() already arrives
    // alliance-resolved from upstream - calling those here would flip it a second time.
    routeBuilder = new FollowPath.Builder(
        driveSubsystem,
        poseEstimator::getEstimatedPose,
        driveSubsystem::getRobotRelativeSpeeds,
        driveSubsystem::driveRobotRelative,
        new PIDController(
            Constants.BLineAuto.TRANSLATION_KP, Constants.BLineAuto.TRANSLATION_KI, Constants.BLineAuto.TRANSLATION_KD),
        new PIDController(
            Constants.BLineAuto.ROTATION_KP, Constants.BLineAuto.ROTATION_KI, Constants.BLineAuto.ROTATION_KD),
        new PIDController(
            Constants.BLineAuto.CROSS_TRACK_KP, Constants.BLineAuto.CROSS_TRACK_KI, Constants.BLineAuto.CROSS_TRACK_KD));

    FollowPath.setDoubleLoggingConsumer(value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setBooleanLoggingConsumer(value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setPoseLoggingConsumer(value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
  }

  @Override
  public Command createRouteCommand(RouteRequest request) {
    RouteConstraints limits = request.constraints();
    Path.PathConstraints constraints = new Path.PathConstraints()
        .setMaxVelocityMetersPerSec(limits.maxVelocityMps())
        .setMaxAccelerationMetersPerSec2(limits.maxAccelerationMps2())
        .setMaxVelocityDegPerSec(Math.toDegrees(limits.maxAngularVelocityRadPerSec()))
        .setMaxAccelerationDegPerSec2(Math.toDegrees(limits.maxAngularAccelerationRadPerSec2()));

    Path path = new Path(constraints, new Path.Waypoint(request.targetPose()));
    return routeBuilder.build(path);
  }
}
