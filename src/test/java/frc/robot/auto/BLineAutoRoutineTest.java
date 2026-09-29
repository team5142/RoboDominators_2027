package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

// Unit tests for BLineAutoRoutine's blue/red starting-pose mirroring.
class BLineAutoRoutineTest {

  private static final double DELTA = 1e-9;

  @Test
  void blueAllianceReturnsStartPoseUnchanged() {
    Pose2d startPoseBlue = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45.0));
    BLineAutoRoutine routine = new BLineAutoRoutine("Test", Commands.none(), startPoseBlue);

    Pose2d result = routine.startPose(false);

    assertEquals(startPoseBlue.getX(), result.getX(), DELTA);
    assertEquals(startPoseBlue.getY(), result.getY(), DELTA);
    assertEquals(startPoseBlue.getRotation().getDegrees(), result.getRotation().getDegrees(), DELTA);
  }

  @Test
  void redAllianceMirrorsStartPose() {
    Pose2d startPoseBlue = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    BLineAutoRoutine routine = new BLineAutoRoutine("Test", Commands.none(), startPoseBlue);

    Pose2d result = routine.startPose(true);

    assertEquals(Constants.Field.FIELD_LENGTH_METERS, result.getX(), DELTA);
    assertEquals(Constants.Field.FIELD_WIDTH_METERS, result.getY(), DELTA);
    assertEquals(180.0, result.getRotation().getDegrees(), DELTA);
  }
}
