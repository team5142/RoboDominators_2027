package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

// Shared field-mirroring math. Poses are authored on the blue alliance side;
// red alliance poses are a 180 degree rotation around the field center.
public final class FieldUtil {
  private FieldUtil() {}

  public static Pose2d mirrorPoseForRed(Pose2d bluePose) {
    return new Pose2d(
        Constants.Field.FIELD_LENGTH_METERS - bluePose.getX(),
        Constants.Field.FIELD_WIDTH_METERS - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.fromDegrees(180.0)));
  }
}
