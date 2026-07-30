package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.StartingPositions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.SmartDriveToPosition;
import frc.robot.commands.util.SetStartingPoseCommand;
import frc.robot.subsystems.*;
import frc.robot.util.SmartLogger;
import frc.robot.util.TouchscreenInterface;

// Wires up robot hardware, controllers, and commands
// To grab latest 10 logs and delete them: run .\scripts\storelogs.bat
public class RobotContainer {
  // === CONFIGURATION ===
  public static final boolean COMPETITION_MODE = false; // Disable logs/streams for matches
  private static final boolean ENABLE_CONSOLE_LOGGING = !COMPETITION_MODE;
  private static final boolean USE_TOUCHSCREEN_OPERATOR = false;
  private static final boolean SYSID_MODE = false; // Phoenix Tuner X characterization mode
  private static final boolean ENABLE_CLIMBER = false;
  private static final boolean ENABLE_SPINDEXER = true;
  private static final boolean ENABLE_SINGULATOR = true;
  private static final double AUTO_SEED_POS_TOL_METERS = 0.20;
  private static final double AUTO_SEED_ROT_TOL_DEG = 10.0;

  private static Alliance cachedAlliance = Alliance.Blue;

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
  ClimberSubsystem climberSubsystem;
  SpindexerSubsystem spindexerSubsystem;
  SingulatorSubsystem singulatorSubsystem;

  // Autonomous chooser shown on dashboard; selection drives pose preview and auto init
  private final SendableChooser<Command> autoChooser;
  private Command lastSelectedAuto = null; // Track selection for preview updates

  // Shot seed pose chooser — shown in Elastic as a dropdown; START button reads the selection.
  private final SendableChooser<Pose2d> shotSeedChooser = new SendableChooser<>();

  // Preview thread writes these; main thread reads them via applyPendingAutoPreviewPose()
  // volatile ensures changes are visible across threads without synchronization
  private volatile Pose2d pendingAutoPreviewPose = null;
  private volatile String pendingAutoPreviewName = null;

  // Last values written to the robot - used to skip redundant resets
  private Pose2d lastAppliedPreviewPose = null;
  private String lastAppliedPreviewName = null;

  private volatile boolean previewThreadRunning = true;

  // Force preview thread to reapply pose/orientation on every boot
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
    //climberSubsystem = ENABLE_CLIMBER ? new ClimberSubsystem(this.robotState) : null;
    spindexerSubsystem = ENABLE_SPINDEXER ? new SpindexerSubsystem(this.robotState) : null;
    singulatorSubsystem = ENABLE_SINGULATOR ? new SingulatorSubsystem(this.robotState) : null;
    if (!ENABLE_CLIMBER) {
      SmartLogger.logConsole("Climber disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_SPINDEXER) {
      SmartLogger.logConsole("Spindexer disabled until hardware is ready", "Startup");
    }
    if (!ENABLE_SINGULATOR) {
      SmartLogger.logConsole("Singulator disabled until hardware is ready", "Startup");
    }

    updateAllianceFromDriverStation();
    this.robotState.setAlliance(cachedAlliance); // Must be set before PathPlanner config

    if (COMPETITION_MODE) {
      SmartLogger.logReplay("Robot/CompetitionMode", true);
    }

    poseEstimator.setTagVisionSubsystem(tagVisionSubsystem); // Cross-wire vision into pose estimator
    SmartDriveToPosition.configure(poseEstimator, robotState, driveSubsystem, questNav); // Static config for SmartDrive commands

    configurePathPlanner();
    configureDefaultCommands();
    configureAlwaysActiveBindings();
    configureButtonBindings();
    
    if (USE_TOUCHSCREEN_OPERATOR) {
      configureTouchscreenInterface();
    }
    
    autoChooser = AutoBuilder.buildAutoChooser(""); // Scans deploy/pathplanner/autos/ for named autos
    autoChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
    SmartDashboard.putData("Auto Chooser", autoChooser); // Sends chooser widget to dashboard
    robotState.setSysIdMode(SYSID_MODE);
    poseEstimator.setAutoChooser(autoChooser); // Lets pose estimator read auto start poses
    startAutoPreviewMonitor(); // Background thread: watches chooser and queues pose previews

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
        .onTrue(driveSubsystem.createOrientToFieldCommand(robotState));

    // START: Seed pose selected from "Shot Seed Pose" dropdown in Elastic.
    // Poses are defined in blue coordinates — flipped automatically when on red alliance.
    // isRed is derived from the final seeded pose X (> field midpoint = red) rather than
    // cachedAlliance, because the cache may not have updated yet when the button is pressed.
    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> {
          Pose2d seed = shotSeedChooser.getSelected();
          if (seed == null) seed = Constants.StartingPositions.SHOT_SEED_2M;
          boolean seedIsRed = isRedAlliance();
          if (seedIsRed) {
            seed = new Pose2d(
                Constants.Field.FIELD_LENGTH_METERS - seed.getX(),
                Constants.Field.FIELD_WIDTH_METERS  - seed.getY(),
                seed.getRotation().plus(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180.0)));
          }
          // Determine perspective from the final pose X so it's correct even if cachedAlliance
          // hasn't updated yet. X > midpoint means the robot is on the red side of the field.
          boolean poseIsRed = seed.getX() > Constants.Field.FIELD_LENGTH_METERS / 2.0;
          new SetStartingPoseCommand(seed, "SHOT SEED", gyro, questNav, driveSubsystem, poseEstimator, poseIsRed)
              .schedule();
        }));

    // D-PAD DOWN: Toggle QuestNav emergency mode.
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(Commands.runOnce(this::toggleQuestNavEmergencyMode));

    // ========== NORMAL OPERATION BUTTONS (COMMENT OUT FOR SYSID) ==========

    // Reusable gate — all pose-dependent commands check this before starting.

    // BOTH TRIGGERS: Dynamic heading snap — gyro only, safe in emergency mode (no gate needed).
    // Holds a field-relative heading based on zone/position while driver steers with left stick.

    // ========== END NORMAL OPERATION BUTTONS ==========

    // ========== OPERATOR CONTROLLER BINDINGS ==========
    // Right stick pulled down (Y > 0.9): reverse singulator and spindexer to clear a jam. Stops on release.
    new Trigger(() -> operatorController.getRightY() > 0.9)
        .whileTrue(Commands.startEnd(
          () -> {
            if (singulatorSubsystem != null) singulatorSubsystem.spinReverse();
            if (spindexerSubsystem  != null) spindexerSubsystem.spinReverse();
          },
          () -> {
            if (singulatorSubsystem != null) singulatorSubsystem.pause();
            if (spindexerSubsystem  != null) spindexerSubsystem.stop();
          }));

    // ========== END OPERATOR CONTROLLER BINDINGS ==========
  }

  private void configureAlwaysActiveBindings() {
  }

  // HTML touchscreen interface
  private void configureTouchscreenInterface() {
    touchscreen = new TouchscreenInterface(robotState, driveSubsystem, poseEstimator, questNav);
    touchscreen.configure();
  }

  // Called when auto starts
  public Command getAutonomousCommand() { 
    Command selectedAuto = autoChooser.getSelected();
    return (selectedAuto != null) ? wrapPathWithLogging(selectedAuto) : selectedAuto;
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

  // Add start/end logging to auto paths
  private Command wrapPathWithLogging(Command pathCommand) {
    String pathName = (pathCommand.getName() != null && !pathCommand.getName().isEmpty()) 
        ? pathCommand.getName() 
        : "Unknown Path";
    
    final String finalPathName = pathName;
    
    return pathCommand
        .beforeStarting(() -> {
          Pose2d startPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + finalPathName + " | Start: " + SmartLogger.formatPose(startPose), "Path Start");
          SmartLogger.logReplay("Auto/CurrentSegment", finalPathName);
          SmartLogger.logReplay("Auto/SegmentStart", startPose);
        })
        .finallyDo((interrupted) -> {
          Pose2d endPose = poseEstimator.getEstimatedPose();
          SmartLogger.logConsole("Segment: " + finalPathName + " | End: " + SmartLogger.formatPose(endPose) + " | Interrupted: " + interrupted, "Path End");
          SmartLogger.logReplay("Auto/SegmentEnd", endPose);
          SmartLogger.logReplay("Auto/SegmentInterrupted", interrupted);
        });
  }

  // Mirror red alliance paths
  private boolean shouldFlipPath() {
    return isRedAlliance();
  }

  // Runs at 2Hz as a daemon thread while disabled.
  // Writes pendingAutoPreviewPose/Name (volatile) when the selected auto changes.
  // The main thread reads them in periodic() via applyPendingAutoPreviewPose().
  private void startAutoPreviewMonitor() {
    Thread previewThread = new Thread(() -> {
      while (previewThreadRunning && !Thread.currentThread().isInterrupted()) {
        try {
          if (DriverStation.isDisabled()) {
            Command selectedAuto = autoChooser.getSelected();

            // On first pass after boot, always apply even if auto hasn't changed.
            // This ensures orientation is correct regardless of prior cached state.
            if (selectedAuto != null && (selectedAuto != lastSelectedAuto || !bootPreviewApplied)) {
              lastSelectedAuto = selectedAuto;
              bootPreviewApplied = true;
              String autoName = selectedAuto.getName();

              Pose2d startingPose = poseEstimator
                  .getPoseInitializer()
                  .getStartPoseForAutoName(autoName);

              pendingAutoPreviewPose = startingPose;
              pendingAutoPreviewName = autoName;
            }
          }
          Thread.sleep(500);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          SmartLogger.logConsole("[Auto Preview] Thread interrupted - stopping");
          break;
        } catch (Exception e) {
          SmartLogger.logConsoleError("[Auto Preview] Error: " + e.getMessage());
        }
      }
      SmartLogger.logConsole("[Auto Preview] Thread stopped cleanly");
    });

    previewThread.setDaemon(true); // Daemon thread - dies automatically when robot code exits
    previewThread.setName("AutoPreview");
    previewThread.start();
  }

  public void periodic() {
    periodicCounter++;
    updateAllianceFromDriverStation();
    robotState.setAlliance(cachedAlliance);
    if (DriverStation.isDisabled()) {
      applyPendingAutoPreviewPose();
    }

    // #1: Controller disconnect alerts — shown in DS and AdvantageScope
    driverDisconnected.set(!DriverStation.isJoystickConnected(driverController.getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operatorController.getPort()));

    // Publish slow-changing fields at 10Hz - alliance/station/auto don't change every loop
    if (periodicCounter % 5 == 0) {
      SmartDashboard.putBoolean("Robot/IsRedAlliance", cachedAlliance == Alliance.Red);
      SmartDashboard.putNumber("Robot/StationNumber", DriverStation.getLocation().orElse(1));
      // Read the chooser's active option name from SmartDashboard - getSelected().getName() returns
      // the Java class name, not the auto name, so we read the NT entry directly
      String activeAuto = SmartDashboard.getString("Auto Chooser/active", "None");
      SmartDashboard.putString("Robot/AutoSelected", activeAuto);
    }
  }

  public static Alliance getAlliance() {
    return cachedAlliance;
  }

  public static boolean isRedAlliance() {
    return cachedAlliance == Alliance.Red;
  }

  private void updateAllianceFromDriverStation() {
    Optional<Alliance> fmsAlliance = DriverStation.getAlliance();
    if (fmsAlliance.isPresent()) {
      cachedAlliance = fmsAlliance.get();
    } else {
      // No FMS — infer from seeded pose X. Robot on Red side has X > field midpoint.
      double midX = Constants.Field.FIELD_LENGTH_METERS / 2.0;
      double poseX = poseEstimator.getEstimatedPose().getX();
      cachedAlliance = (poseX > midX) ? Alliance.Red : Alliance.Blue;
    }
  }

  // Called from periodic() (main thread) to apply a pose queued by the preview thread.
  // Uses volatile reads - no locks needed because Pose2d is immutable.
  private void applyPendingAutoPreviewPose() {
    Pose2d pose = pendingAutoPreviewPose;
    String autoName = pendingAutoPreviewName;

    if (pose == null || autoName == null) {
      return;
    }

    if (pose.equals(lastAppliedPreviewPose) && autoName.equals(lastAppliedPreviewName)) {
      return;
    }

    pendingAutoPreviewPose = null;
    pendingAutoPreviewName = null;

    lastAppliedPreviewPose = pose;
    lastAppliedPreviewName = autoName;

    // Seed the gyro to the auto start pose's field heading, then set perspective = allianceDownfield.
    // CTRE field-centric: effectiveHeading = gyro - perspective, so forward = Red wall.
    boolean isRedForPreview = cachedAlliance == Alliance.Red;
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

    SmartLogger.logConsole("Auto: " + autoName + " | Pose: " + SmartLogger.formatPose(pose), "Preview");
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

  private Pose2d getRebuiltRightCornerPose() {
    if (!isRedAlliance()) return BLUE_REBUILT_RIGHT_CORNER;
    // On a full competition field use the mirrored far corner.
    // On the practice field use the dedicated Red seed pose instead.
    return COMPETITION_MODE ? RED_REBUILT_RIGHT_CORNER : RED_PRACTICE_SEED;
  }
}

