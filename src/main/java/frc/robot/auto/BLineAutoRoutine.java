package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldUtil;

// One named autonomous routine: the command that runs it, and its blue-alliance
// starting pose (paths are authored on the blue side, same convention PathPlanner's
// stored autos used).
public record BLineAutoRoutine(String name, Command command, Pose2d startPoseBlue) {

  // Alliance-corrected starting pose, using the same team-owned mirroring math
  // PoseInitializer has always used for stored-auto starting poses.
  public Pose2d startPose(boolean isRed) {
    return isRed ? FieldUtil.mirrorPoseForRed(startPoseBlue) : startPoseBlue;
  }
}
