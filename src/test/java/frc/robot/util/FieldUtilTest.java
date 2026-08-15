package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class FieldUtilTest {

  private static final double DELTA = 1e-9;

  @Test
  void mirrorsOriginToFarCornerFacingBackward() {
    Pose2d mirrored = FieldUtil.mirrorPoseForRed(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    assertEquals(Constants.Field.FIELD_LENGTH_METERS, mirrored.getX(), DELTA);
    assertEquals(Constants.Field.FIELD_WIDTH_METERS, mirrored.getY(), DELTA);
    assertEquals(180.0, mirrored.getRotation().getDegrees(), DELTA);
  }

  @Test
  void wrapsRotationCorrectly() {
    // 180 + 180 = 360, which Rotation2d normalizes to 0.
    Pose2d mirrored = FieldUtil.mirrorPoseForRed(
        new Pose2d(Constants.Field.FIELD_LENGTH_METERS, Constants.Field.FIELD_WIDTH_METERS, Rotation2d.fromDegrees(180.0)));

    assertEquals(0.0, mirrored.getX(), DELTA);
    assertEquals(0.0, mirrored.getY(), DELTA);
    assertEquals(0.0, mirrored.getRotation().getDegrees(), DELTA);
  }

  @Test
  void mirroringTwiceReturnsToOriginalPose() {
    Pose2d original = new Pose2d(3.62, 2.515, Rotation2d.fromDegrees(37.0));

    Pose2d roundTripped = FieldUtil.mirrorPoseForRed(FieldUtil.mirrorPoseForRed(original));

    assertEquals(original.getX(), roundTripped.getX(), DELTA);
    assertEquals(original.getY(), roundTripped.getY(), DELTA);
    assertEquals(original.getRotation().getDegrees(), roundTripped.getRotation().getDegrees(), DELTA);
  }
}
