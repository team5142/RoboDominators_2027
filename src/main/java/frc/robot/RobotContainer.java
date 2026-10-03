package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.auto.BLineAutoRoutines;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.route.PathPlannerCoarseRouteProvider;
import frc.robot.subsystems.*;
import frc.robot.util.FieldUtil;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

// Wires up robot hardware, controllers, and commands
// To grab latest 10 logs and delete them: run .\scripts\storelogs.bat
public class RobotContainer {
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = false;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  // Shown as the "no auto" option in the dashboard chooser - must match exactly
  // everywhere it is compared, since the chooser stores plain routine-name strings.
  private static final String DO_NOTHING_AUTO_NAME = "Do Nothing";
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;

  // Driver Xbox controller on USB port defined in Constants
  private final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
  // Operator Xbox controller on the next USB slot (port 1)
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final Alert driverDisconnected   = new Alert("Driver controller disconnected (port " + DRIVER_CONTROLLER_PORT + ").", AlertType.kWarning);
  private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port " + Constants.OPERATOR_CONTROLLER_PORT + ").", AlertType.kWarning);

  // When REQUIRE_TURRET_FORWARD_CONFIRM=true, bindings are deferred until Back+A confirm.
  private boolean bindingsConfigured = false;

  // Subsystems - order here matches initialization order in constructor
  final RobotState robotState;
  final GyroSubsystem gyro;
  final QuestNavSubsystem questNav;
  final DriveSubsystem driveSubsystem;
  final PoseEstimatorSubsystem poseEstimator;
  final TagVisionSubsystem tagVisionSubsystem;
  public final LEDSubsystem ledSubsystem;
  final SmartDriveToPosition smartDriveToPosition;
  final BLineAutoRoutines blineAutoRoutines;

  // Autonomous chooser shown on dashboard; selection drives pose preview and auto init
  private final SendableChooser<String> autoChooser;
  private String lastSelectedAuto = null; // Track selection for preview updates

  // Shot seed pose chooser — shown in Elastic as a dropdown; START button reads the selection.
  private final SendableChooser<Pose2d> shotSeedChooser = new SendableChooser<>();

  // Force the auto preview to reapply pose/orientation once on every boot
  private boolean bootPreviewApplied = false;

  private TouchscreenInterface touchscreen;
  private int periodicCounter = 0;

  // === CONSTRUCTOR - Runs once at robot boot ===
  public RobotContainer(RobotState robotState) {
    this.robotState = robotState;

    SmartLogger.configure(ENABLE_CONSOLE_LOGGING); // Configure first so all init logs work

    gyro = new GyroSubsystem();
    questNav = new QuestNavSubsystem();
    driveSubsystem = new DriveSubsystem(this.robotState, gyro);
    poseEstimator = new PoseEstimatorSubsystem(driveSubsystem, this.robotState, questNav);
    tagVisionSubsystem = new TagVisionSubsystem(poseEstimator);
    ledSubsystem = new LEDSubsystem(this.robotState);

    updateAllianceFromDriverStation(); // Sets robotState's alliance - must happen before PathPlanner config

    if (COMPETITION_MODE) {
      SmartLogger.logReplay("Robot/CompetitionMode", true);
    }

    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem); // Cross-wire vision into pose estimator
    smartDriveToPosition = new SmartDriveToPosition(
        poseEstimator, robotState, driveSubsystem, questNav, new PathPlannerCoarseRouteProvider());

    configurePathPlanner();
    configureDefaultCommands();
    configureAlwaysActiveBindings();
    configureButtonBindings();
    
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    // Builds every named BLine autonomous routine once at boot (paths, event triggers,
    // starting poses). See BLineAutoRoutines for what a routine actually is.
    blineAutoRoutines = new BLineAutoRoutines(driveSubsystem, poseEstimator, ledSubsystem);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption(DO_NOTHING_AUTO_NAME, DO_NOTHING_AUTO_NAME);
    for (String routineName : blineAutoRoutines.names()) {
      autoChooser.addOption(routineName, routineName);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser); // Sends chooser widget to dashboard
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser); // Lets pose estimator read auto start poses
    poseEstimator.getPoseInitializer().setAutoRoutines(blineAutoRoutines); // Lets pose estimator look up start poses by routine name

    shotSeedChooser.setDefaultOption("HUBCLOSE (1.28m)", Constants.StartingPositions.SHOT_SEED_HUBCLOSE);
    shotSeedChooser.addOption("HUB 1.7M (1.67m)", Constants.StartingPositions.SHOT_SEED_HUB1_7M);
    shotSeedChooser.addOption("HUB RIGHT ACCURATE (1.07m)", Constants.StartingPositions.SHOT_SEED_HUB_RIGHT_ACCURATE);
    shotSeedChooser.addOption("RIGHT BUMP (2.03m)", Constants.StartingPositions.SHOT_SEED_RIGHT_BUMP);
    shotSeedChooser.addOption("LEFT BUMP (2.03m)",  Constants.StartingPositions.SHOT_SEED_LEFT_BUMP);
    shotSeedChooser.addOption("2M",   Constants.StartingPositions.SHOT_SEED_2M);
    shotSeedChooser.addOption("2.5M", Constants.StartingPositions.SHOT_SEED_2_5M);
    shotSeedChooser.addOption("3M",   Constants.StartingPositions.SHOT_SEED_3M);
    shotSeedChooser.addOption("4M",   Constants.StartingPositions.SHOT_SEED_4M);
    shotSeedChooser.addOption("4.5M", Constants.StartingPositions.SHOT_SEED_4_5M);
    shotSeedChooser.addOption("OUTPOST (5.48m)", Constants.StartingPositions.SHOT_SEED_OUTPOST);
    shotSeedChooser.addOption("BACK WALL RIGHT (4.59m)", Constants.StartingPositions.SHOT_SEED_BACK_WALL_RIGHT);
    shotSeedChooser.addOption("RIGHT CORNER (4.59m)", Constants.StartingPositions.SHOT_SEED_RIGHT_CORNER);
    SmartDashboard.putData("Shot Seed Pose", shotSeedChooser);

    SmartLogger.logConsole("RobotContainer initialized - all subsystems ready", "Init Complete", 5);
  }

  // Configure PathPlanner auto builder
  // Connects PathPlanner to this robot's drive system.
  // feedforwards are intentionally ignored - we use odometry-only closed-loop control.
  // Translation/rotation PID constants are tuned in Constants.java.
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
          poseEstimator::getEstimatedPose,
          this::resetPose,
          driveSubsystem::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds), // feedforwards unused
          new PPHolonomicDriveController(
              new PIDConstants(
          TRANSLATION_KP,
          TRANSLATION_KI,
          TRANSLATION_KD),
              new PIDConstants(
          ROTATION_KP,
          ROTATION_KI,
          ROTATION_KD)),
          config,
          this::shouldFlipPath,
          driveSubsystem);

      /*
       * Send PathPlanner's internal values to AdvantageKit so we can tune its PID in
       * AdvantageScope. Every loop while a path runs, PathPlanner reports the pose it
       * wants the robot at right now (TargetPose). The errors are how far the robot's
       * real pose is from that target - these are exactly what the translation and
       * rotation PIDs are trying to push to zero.
       */
      PathPlannerLogging.setLogActivePathCallback(
          poses -> Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0])));
      PathPlannerLogging.setLogTargetPoseCallback(target -> {
        Pose2d current = poseEstimator.getEstimatedPose();
        Logger.recordOutput("PathPlanner/TargetPose", target);
        Logger.recordOutput("PathPlanner/TranslationErrorMeters",
            target.getTranslation().getDistance(current.getTranslation()));
        Logger.recordOutput("PathPlanner/RotationErrorDegrees",
            target.getRotation().minus(current.getRotation()).getDegrees());
      });

      SmartLogger.logConsole("PathPlanner configured - PID tunable in AdvantageScope", "PathPlanner");
    } catch (Exception e) {
      SmartLogger.logConsoleError("PathPlanner config failed: " + e.getMessage());
      DriverStation.reportWarning("PathPlanner config failed!", false);
    }
  }

  // Set default commands (run when subsystems idle)
  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        new DriveWithJoysticks(
            driveSubsystem, robotState,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true,
      () -> false)); // Left bumper reserved for bump traversal.
  }

  // Map controller buttons to commands
  private void configureButtonBindings() {
    bindingsConfigured = true;

    // ========== UTILITY BUTTONS (ALWAYS ACTIVE) ==========

    // BACK: Reset field orientation
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(driveSubsystem.createOrientToFieldCommand());

    // START: Seed pose selected from "Shot Seed Pose" dropdown in Elastic.
    // Poses are defined in blue coordinates — flipped automatically when on red alliance.
    // isRed is derived from the final seeded pose X (> field midpoint = red) rather than
    // robotState.getAlliance(), because that may not have updated yet when the button is pressed.
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          Pose2d seed = shotSeedChooser.getSelected();
          if (seed == null) seed = Constants.StartingPositions.SHOT_SEED_2M;
          boolean seedIsRed = robotState.getAlliance() == Alliance.Red;
          if (seedIsRed) {
            seed = FieldUtil.mirrorPoseForRed(seed);
          }
          // Determine perspective from the final pose X so it's correct even if robotState's
          // alliance hasn't updated yet. X > midpoint means the robot is on the red side of the field.
          boolean poseIsRed = seed.getX() > Constants.Field.FIELD_LENGTH_METERS / 2.0;
          CommandScheduler.getInstance().schedule(
              new SetStartingPoseCommand(seed, "SHOT SEED", gyro, questNav, driveSubsystem, poseEstimator, poseIsRed));
        }));

    // D-PAD DOWN: Toggle QuestNav emergency mode.
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(Commands.runOnce(this::toggleQuestNavEmergencyMode));

    // ========== NORMAL OPERATION BUTTONS (COMMENT OUT FOR SYSID) ==========

    // Reusable gate — all pose-dependent commands check this before starting.

    // BOTH TRIGGERS: Dynamic heading snap — gyro only, safe in emergency mode (no gate needed).
    // Holds a field-relative heading based on zone/position while driver steers with left stick.

    /*
     * X and B: hold-to-drive to a fixed shooting pose near the blue drive-team area,
     * facing the hub, using SmartDriveToPosition. X is the left face button, B is the
     * right face button on a standard Xbox controller, matching "left pose" /
     * "right pose" below.
     *
     * SmartDriveToPosition takes two poses and drives in two phases. First,
     * PathPlanner moves quickly (but not very accurately) to the staging pose, which
     * sits 0.62m back from the real target. Then AutoPilot takes over and creeps from
     * the staging pose to the precise pose slowly and accurately. Passing the same
     * pose twice would skip that handoff, so every target needs its own staging pose.
     *
     * whileTrue schedules the drive while the button is held and cancels it the
     * instant the button is released - and the command also ends on its own once
     * SmartDriveToPosition reports it has reached the target pose, whichever comes
     * first. Commands.defer builds a brand new SmartDriveToPosition command every
     * time the button is pressed, so its internal state (QuestNav lock flags, etc.)
     * never carries over from a previous press.
     *
     * The two poses are real, measured field positions (Constants.StartingPositions),
     * authored in blue-alliance coordinates. allianceAdjustedPose() mirrors them for
     * red alliance the same way the START button's shot-seed handler already does.
     */
    new JoystickButton(driverController, XboxController.Button.kX.value)
        .whileTrue(Commands.defer(
            () -> smartDriveToPosition.create(
                allianceAdjustedPose(Constants.StartingPositions.SHOT_SEED_LEFT_BUMP_STAGING),
                allianceAdjustedPose(Constants.StartingPositions.SHOT_SEED_LEFT_BUMP)),
            Set.of(driveSubsystem)));

    new JoystickButton(driverController, XboxController.Button.kB.value)
        .whileTrue(Commands.defer(
            () -> smartDriveToPosition.create(
                allianceAdjustedPose(Constants.StartingPositions.SHOT_SEED_RIGHT_BUMP_STAGING),
                allianceAdjustedPose(Constants.StartingPositions.SHOT_SEED_RIGHT_BUMP)),
            Set.of(driveSubsystem)));

    // Y: same hold-to-drive pattern, to the ETHAN_TEST pose (a test target, not a game position).
    new JoystickButton(driverController, XboxController.Button.kY.value)
        .whileTrue(Commands.defer(
            () -> smartDriveToPosition.create(
                allianceAdjustedPose(Constants.StartingPositions.ETHAN_TEST_STAGING),
                allianceAdjustedPose(Constants.StartingPositions.ETHAN_TEST)),
            Set.of(driveSubsystem)));

    // ========== END NORMAL OPERATION BUTTONS ==========

  }

  private void configureAlwaysActiveBindings() {
  }

  // HTML touchscreen interface
  private void configureTouchscreenInterface() {
    touchscreen = new TouchscreenInterface(robotState, driveSubsystem, poseEstimator, questNav, smartDriveToPosition);
    touchscreen.configure();
  }

  // Called when auto starts
  public Command getAutonomousCommand() {
    String selectedName = autoChooser.getSelected();
    if (selectedName == null || DO_NOTHING_AUTO_NAME.equals(selectedName)) {
      return Commands.none();
    }
    return wrapPathWithLogging(selectedName, blineAutoRoutines.commandFor(selectedName));
  }

  public void onTeleopInit() {
  }

  public void toggleQuestNavEmergencyMode() {
    boolean nowActive = !robotState.isQuestNavEmergencyMode();
    robotState.setQuestNavEmergencyMode(nowActive);
  }

  // Reset robot position
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose, driveSubsystem.getGyroRotation(), driveSubsystem.getModulePositions());
    SmartLogger.logConsole("Pose reset to: " + SmartLogger.formatPose(pose));
  }

  // Poses in Constants.StartingPositions are authored in blue-alliance coordinates.
  // Mirror to red the same way the START button's shot-seed handler already does.
  private Pose2d allianceAdjustedPose(Pose2d bluePose) {
    return robotState.getAlliance() == Alliance.Red ? FieldUtil.mirrorPoseForRed(bluePose) : bluePose;
  }

  // Add start/end logging to a BLine autonomous routine's command
  private Command wrapPathWithLogging(String routineName, Command routineCommand) {
    return routineCommand
        .beforeStarting(() -> {
          Pose2d startPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + routineName + " | Start: " + SmartLogger.formatPose(startPose), "Path Start");
          SmartLogger.logReplay("Auto/CurrentSegment", routineName);
          SmartLogger.logReplay("Auto/SegmentStart", startPose);
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + routineName + " | End: " + SmartLogger.formatPose(endPose) + " | Interrupted: " + interrupted, "Path End");
          SmartLogger.logReplay("Auto/SegmentEnd", endPose);
          SmartLogger.logReplay("Auto/SegmentInterrupted", interrupted);
        });
  }

  // Mirror red alliance paths
  private boolean shouldFlipPath() {
    return robotState.getAlliance() == Alliance.Red;
  }

  public void periodic() {
    periodicCounter++;
    updateAllianceFromDriverStation();
    if (DriverStation.isDisabled()) {
      updateAutoPreview();
    }

    // #1: Controller disconnect alerts — shown in DS and AdvantageScope
    driverDisconnected.set(!DriverStation.isJoystickConnected(driverController.getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operatorController.getPort()));

    // Publish slow-changing fields at 10Hz - alliance/station/auto don't change every loop
    if (periodicCounter % 5 == 0) {
      SmartDashboard.putBoolean("Robot/IsRedAlliance", robotState.getAlliance() == Alliance.Red);
      SmartDashboard.putNumber("Robot/StationNumber", DriverStation.getLocation().orElse(1));
      // Read the chooser's active option name from SmartDashboard - getSelected().getName() returns
      // the Java class name, not the auto name, so we read the NT entry directly
      String activeAuto = SmartDashboard.getString("Auto Chooser/active", "None");
      SmartDashboard.putString("Robot/AutoSelected", activeAuto);
    }
  }

  // Alliance comes straight from the Driver Station app's station selector (Blue/Red 1-3),
  // which is set manually during practice and by FMS during a real match - no FMS
  // connection is required for the DS app to report it. Defaults to Blue only if the DS
  // hasn't reported anything at all yet (e.g. very early in boot).
  private void updateAllianceFromDriverStation() {
    robotState.setAlliance(DriverStation.getAlliance().orElse(Alliance.Blue));
  }

  // Called from periodic() (main loop) while disabled. Detects an auto chooser change and
  // re-seeds gyro/pose/QuestNav to match - keeps the pose estimate aligned with whichever
  // auto is selected so autonomous never starts from a stale pose. Runs on the main loop
  // directly (not a background thread) so there's no latency window between the driver
  // changing the dropdown and the pose actually updating.
  private void updateAutoPreview() {
    String selectedName = autoChooser.getSelected();

    // On first pass after boot, always apply even if the auto hasn't changed, so
    // orientation is correct regardless of prior state.
    if (selectedName == null || (selectedName.equals(lastSelectedAuto) && bootPreviewApplied)) {
      return;
    }
    lastSelectedAuto = selectedName;
    bootPreviewApplied = true;

    Pose2d pose = poseEstimator.getPoseInitializer().getStartPoseForAutoName(selectedName);
    if (pose == null) {
      return;
    }

    // Seed the gyro to the auto start pose's field heading, then set perspective = allianceDownfield.
    // CTRE field-centric: effectiveHeading = gyro - perspective, so forward = Red wall.
    boolean isRedForPreview = robotState.getAlliance() == Alliance.Red;
    Rotation2d allianceDownfield = Rotation2d.fromDegrees(isRedForPreview ? 180.0 : 0.0);
    Rotation2d poseHeading = pose.getRotation();
    driveSubsystem.setGyroHeading(poseHeading);
    driveSubsystem.setOperatorPerspectiveForward(allianceDownfield);

    poseEstimator.resetPose(pose, poseHeading, driveSubsystem.getModulePositions());

    SmartLogger.logConsole(
        "Preview orient: alliance=" + (isRedForPreview ? "Red" : "Blue")
        + " | poseHeading=" + String.format("%.1f", poseHeading.getDegrees())
        + " | perspective=" + String.format("%.1f", allianceDownfield.getDegrees()),
        "Preview");

    if (questNavNeedsSeed(pose)) {
      questNav.seedToPose(pose);
    }

    SmartLogger.logConsole("Auto: " + selectedName + " | Pose: " + SmartLogger.formatPose(pose), "Preview");
    SmartLogger.logReplay("Auto/PreviewPose", pose);
  }

  // Returns true if QuestNav's current pose is far enough from desiredPose to need re-seeding.
  // Tolerances are defined in Constants to avoid seeding on minor drift.
  private boolean questNavNeedsSeed(Pose2d desiredPose) {
    if (!questNav.isTracking()) {
      return false;
    }

    Pose2d qPose = questNav.getRobotPose().orElse(null);
    if (qPose == null) {
      return true;
    }

    double posErr = qPose.getTranslation().getDistance(desiredPose.getTranslation());
    double rotErr = Math.abs(qPose.getRotation().minus(desiredPose.getRotation()).getDegrees());

    SmartLogger.logReplay("Auto/QuestPoseErrMeters", posErr);
    SmartLogger.logReplay("Auto/QuestRotErrDeg", rotErr);

    return posErr > AUTO_SEED_POS_TOL_METERS || rotErr > AUTO_SEED_ROT_TOL_DEG;
  }
}

