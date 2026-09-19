package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.route.CoarseRouteProvider;
import frc.robot.route.RouteConstraints;
import frc.robot.route.RouteRequest;
import org.junit.jupiter.api.Test;

// Confirms SmartDriveToPosition hands its CoarseRouteProvider the correct staging pose
// and the unchanged fast-approach constraints (3.5 m/s, 3.5 m/s^2, 540 deg/s, 720 deg/s^2).
// Only exercises the fast-approach leg, not the full FAST_APPROACH/PRECISION_PATH/LOCKED
// sequence (which needs live DriveSubsystem/PoseEstimatorSubsystem instances) - that's
// covered by simulation/robot testing, not a unit test.
class SmartDriveToPositionTest {

  private static final double DELTA = 1e-9;

  private static class RecordingRouteProvider implements CoarseRouteProvider {
    RouteRequest lastRequest;

    @Override
    public Command createRouteCommand(RouteRequest request) {
      lastRequest = request;
      return Commands.none();
    }
  }

  @Test
  void sendsCorrectStagingPoseAndConstraintsForTranslationHeavyTarget() {
    RecordingRouteProvider provider = new RecordingRouteProvider();
    SmartDriveToPosition smartDrive =
        new SmartDriveToPosition(null, null, null, null, provider);
    Pose2d stagingPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0.0));

    smartDrive.fastApproachCommand(stagingPose);

    assertEquals(stagingPose, provider.lastRequest.targetPose());
    assertFastApproachConstraints(provider.lastRequest.constraints());
  }

  @Test
  void sendsCorrectStagingPoseAndConstraintsForRotationHeavyTarget() {
    RecordingRouteProvider provider = new RecordingRouteProvider();
    SmartDriveToPosition smartDrive =
        new SmartDriveToPosition(null, null, null, null, provider);
    Pose2d stagingPose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(90.0));

    smartDrive.fastApproachCommand(stagingPose);

    assertEquals(stagingPose, provider.lastRequest.targetPose());
    assertFastApproachConstraints(provider.lastRequest.constraints());
  }

  private static void assertFastApproachConstraints(RouteConstraints constraints) {
    assertEquals(3.5, constraints.maxVelocityMps(), DELTA);
    assertEquals(3.5, constraints.maxAccelerationMps2(), DELTA);
    assertEquals(Math.toRadians(540.0), constraints.maxAngularVelocityRadPerSec(), DELTA);
    assertEquals(Math.toRadians(720.0), constraints.maxAngularAccelerationRadPerSec2(), DELTA);
  }
}
