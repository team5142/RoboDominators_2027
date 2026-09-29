package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Pattern;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

// Registry of BLine-authored autonomous routines. Built once at boot; replaces
// PathPlanner's AutoBuilder.buildAutoChooser() and PoseInitializer's .auto/.path file
// parsing. BLine has no folder-scanning chooser or bundled "auto file" format of its
// own, so routines are named and composed here in Java. PathPlanner keeps its dynamic
// pathfinding role in SmartDriveToPosition - untouched by this class.
public class BLineAutoRoutines {

  private final FollowPath.Builder routeBuilder;
  private final Map<String, BLineAutoRoutine> routines = new LinkedHashMap<>();

  public BLineAutoRoutines(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator, LEDSubsystem ledSubsystem) {
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        Constants.BLineAuto.DEFAULT_MAX_VELOCITY_MPS,
        Constants.BLineAuto.DEFAULT_MAX_ACCELERATION_MPS2,
        Constants.BLineAuto.DEFAULT_MAX_VELOCITY_DEG_PER_SEC,
        Constants.BLineAuto.DEFAULT_MAX_ACCELERATION_DEG_PER_SEC2,
        Constants.BLineAuto.DEFAULT_END_TRANSLATION_TOLERANCE_METERS,
        Constants.BLineAuto.DEFAULT_END_ROTATION_TOLERANCE_DEG,
        Constants.BLineAuto.DEFAULT_INTERMEDIATE_HANDOFF_RADIUS_METERS));

    // One Builder, reused for every routine, so alliance flip and tuning stay consistent.
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
            Constants.BLineAuto.CROSS_TRACK_KP, Constants.BLineAuto.CROSS_TRACK_KI, Constants.BLineAuto.CROSS_TRACK_KD))
        .withDefaultShouldFlip();

    // Real, working event-trigger example: flashes LEDs green. Proves
    // FollowPath.registerEventTrigger end-to-end using a subsystem that actually exists.
    // TODO: register real game-piece event triggers (e.g. "intake", "shoot") once 2027
    // mechanism subsystems exist. Do not leave "signal" as the only real trigger.
    FollowPath.registerEventTrigger("signal", Commands.runOnce(() -> ledSubsystem.setPattern(Pattern.GREEN)));

    registerRoutine("Ethan 2.0", "ethan2point0");
  }

  // Loads a BLine-Web-authored path by name (from deploy/autos/paths/<pathFileName>.json)
  // and registers it as a chooser-visible routine. Add one call like this per real auto -
  // this is where a new path file actually becomes selectable on the dashboard.
  private void registerRoutine(String displayName, String pathFileName) {
    Path path = new Path(pathFileName);
    Command command = BLineCommands.sequence(routeBuilder.build(path));
    routines.put(displayName, new BLineAutoRoutine(displayName, command, path.getStartPose()));
  }

  public List<String> names() {
    return List.copyOf(routines.keySet());
  }

  public Command commandFor(String name) {
    BLineAutoRoutine routine = routines.get(name);
    return routine != null ? routine.command() : Commands.none();
  }

  public Optional<Pose2d> startPose(String name, boolean isRed) {
    BLineAutoRoutine routine = routines.get(name);
    return routine == null ? Optional.empty() : Optional.of(routine.startPose(isRed));
  }
}
