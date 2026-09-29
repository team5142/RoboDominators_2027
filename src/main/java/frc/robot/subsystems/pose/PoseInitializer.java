package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.BLineAutoRoutines;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.util.SmartLogger;
import org.littletonrobotics.junction.Logger;

// Determines initialization mode: COMP_SEED (seed Quest to known start) vs SHOP_RESUME (use Quest's existing tracking)
public class PoseInitializer {
  
  public enum InitializationState {
    WAITING,        // Not yet initialized
    INITIALIZED,    // Successfully initialized (either mode)
    FALLBACK_USED   // Fallback used (not currently implemented)
  }
  
  public static class InitResult {
    public final Pose2d pose;
    public final boolean shouldSeedQuest;
    public final String reason;
    
    public InitResult(Pose2d pose, boolean shouldSeedQuest, String reason) {
      this.pose = pose;
      this.shouldSeedQuest = shouldSeedQuest;
      this.reason = reason;
    }
  }
  
  private final QuestNavSubsystem questNavSubsystem;
  private final Timer initWaitTimer = new Timer();
  
  private InitializationState initState = InitializationState.WAITING;
  private SendableChooser<String> autoChooser;
  // Where BLine autonomous routine names, commands, and starting poses actually live.
  // Set once RobotContainer builds the routines (after this class is constructed), the
  // same "constructed now, wired in a moment later" pattern autoChooser already uses.
  private BLineAutoRoutines autoRoutines;
  private boolean noPoseWarningShown = false;

  private static final double FIELD_LENGTH_METERS = Constants.Field.FIELD_LENGTH_METERS;
  private static final double FIELD_WIDTH_METERS = Constants.Field.FIELD_WIDTH_METERS;
  private static final double FIELD_MARGIN_METERS = 0.3;
  private static final double MAX_SANE_POSE_MAGNITUDE = 100.0; // Sanity check for unanchored poses

  public PoseInitializer(QuestNavSubsystem questNavSubsystem) {
    this.questNavSubsystem = questNavSubsystem;
    initWaitTimer.start();
  }

  public void setAutoChooser(SendableChooser<String> autoChooser) {
    this.autoChooser = autoChooser;
  }

  public void setAutoRoutines(BLineAutoRoutines autoRoutines) {
    this.autoRoutines = autoRoutines;
  }

  // Returns the currently selected auto routine's name, or null if nothing is selected.
  public String getSelectedAutoName() {
    return autoChooser != null ? autoChooser.getSelected() : null;
  }

  public void updateReadiness() {
    Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
    boolean hasQuestNavPose = (questNavPose != null);
    boolean isFMSAttached = DriverStation.isFMSAttached();
    
    if (hasQuestNavPose) {
      if (isFMSAttached) {
        SmartDashboard.putString("Pose/InitStatus", "QuestNav ready - FIELD ALIGNED");
        SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
        SmartDashboard.putBoolean("Pose/FieldAligned", true);
        Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", true);
      } else {
        SmartDashboard.putString("Pose/InitStatus", "QuestNav ready - TELEOP ONLY (not field-aligned)");
        SmartDashboard.putBoolean("Pose/ReadyToEnable", true);
        SmartDashboard.putBoolean("Pose/FieldAligned", false);
        Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", false);
      }
    } else {
      SmartDashboard.putString("Pose/InitStatus", "MANUAL RESET REQUIRED");
      SmartDashboard.putBoolean("Pose/ReadyToEnable", false);
      SmartDashboard.putBoolean("Pose/FieldAligned", false);
      Logger.recordOutput("PoseEstimator/Readiness/FieldAligned", false);
    }
    
    SmartDashboard.putBoolean("Vision/MultiTagReady", false);
    SmartDashboard.putBoolean("Vision/SingleTagReady", false);
    SmartDashboard.putBoolean("QuestNav/Ready", hasQuestNavPose);
    
    Logger.recordOutput("PoseEstimator/Readiness/VisionDisabled", true);
    Logger.recordOutput("PoseEstimator/Readiness/QuestNav", hasQuestNavPose);
  }
  
  public InitResult attemptInitialization() {
    // === COMP_SEED MODE: FMS attached — always seed Quest to auto start pose ===
    if (DriverStation.isDisabled() && DriverStation.isFMSAttached()) {
      Pose2d autoStartPose = getExpectedAutoStartPose();
      if (autoStartPose != null && isWithinField(autoStartPose.getTranslation())) {
        initState = InitializationState.INITIALIZED;
        Logger.recordOutput("PoseEstimator/InitWaitSeconds", initWaitTimer.get());
        Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
        Logger.recordOutput("PoseEstimator/InitMode", "COMP_SEED");
        return new InitResult(autoStartPose, true, "COMP_SEED: Auto start pose from chooser");
      }
    }

    // === SHOP_RESUME / PRACTICE MODE: No FMS ===
    // If Quest is already tracking a sane pose, resume from it — do NOT overwrite with auto start.
    // Only fall back to auto start pose if Quest has no tracking (e.g. first boot, tracker lost).
    if (!DriverStation.isFMSAttached()) {
      Pose2d questNavPose = questNavSubsystem.getRobotPose().orElse(null);
      if (questNavPose != null && isSanePose(questNavPose)) {
        initState = InitializationState.INITIALIZED;
        Logger.recordOutput("PoseEstimator/InitWaitSeconds", initWaitTimer.get());
        Logger.recordOutput("PoseEstimator/InitializedViaQuestNav", true);
        Logger.recordOutput("PoseEstimator/InitMode", "SHOP_RESUME");
        Logger.recordOutput("PoseEstimator/UnanchoredFrame", true);
        SmartLogger.logConsole("SHOP_RESUME: Quest tracking unanchored, pose: " + SmartLogger.formatPose(questNavPose));
        SmartLogger.logConsoleError("WARNING: Not field-aligned - teleop practice only!");
        return new InitResult(questNavPose, false, "SHOP_RESUME: Quest existing tracking (UNANCHORED - teleop only)");
      }

      // Quest not tracking — fall back to auto start pose so the robot at least has a known origin
      if (DriverStation.isDisabled()) {
        Pose2d autoStartPose = getExpectedAutoStartPose();
        if (autoStartPose != null && isWithinField(autoStartPose.getTranslation())) {
          initState = InitializationState.INITIALIZED;
          Logger.recordOutput("PoseEstimator/InitWaitSeconds", initWaitTimer.get());
          Logger.recordOutput("PoseEstimator/InitializedFromAuto", true);
          Logger.recordOutput("PoseEstimator/InitMode", "PRACTICE_AUTO_SEED");
          return new InitResult(autoStartPose, true, "PRACTICE_AUTO_SEED: Quest not tracking, using auto start pose");
        }
      }
    }
    
    SmartDashboard.putString("Pose/InitMethod", "BLOCKED - Quest not tracking");
    
    if (!noPoseWarningShown) {
      SmartLogger.logConsoleError("Cannot initialize: Quest not tracking");
      Logger.recordOutput("PoseEstimator/NoPoseWarningShown", true);
      noPoseWarningShown = true;
    }
    
    return null;
  }
  
  public Pose2d getStartPoseForAutoName(String autoName) {
    // Use the FMS-provided alliance, falling back to Red=false (Blue) only if truly unknown.
    // Callers with a reliable cached alliance should use getStartPoseForAutoName(name, isRed).
    boolean isRed = DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false);
    return getStartPoseForAutoName(autoName, isRed);
  }

  // Preferred overload — pass cachedAlliance from RobotContainer/RobotState so we never
  // default to Blue during the FMS handshake window at match start.
  public Pose2d getStartPoseForAutoName(String autoName, boolean isRed) {
    if (autoName == null || autoName.isEmpty() || autoRoutines == null) return null;

    Pose2d pose = autoRoutines.startPose(autoName, isRed).orElse(null);
    if (pose == null) {
      Logger.recordOutput("PoseInitializer/UnknownAuto", autoName);
      return null;
    }

    Logger.recordOutput("PoseInitializer/AutoStartPose", pose);
    Logger.recordOutput("PoseInitializer/AutoStartPoseFlipped", isRed);
    return pose;
  }

  private Pose2d getExpectedAutoStartPose() {
    if (autoChooser == null) return null;

    try {
      String autoName = autoChooser.getSelected();
      if (autoName == null) return null;

      Pose2d pose = getStartPoseForAutoName(autoName);

      if (pose == null) {
        Logger.recordOutput("PoseInitializer/UnknownAuto", autoName);
      }

      return pose;
    } catch (Exception e) {
      Logger.recordOutput("PoseInitializer/GetPoseError", e.getMessage());
      return null;
    }
  }
  
  // Field bounds check (only for COMP_SEED mode - known field frame)
  private boolean isWithinField(Translation2d point) {
    return point.getX() > FIELD_MARGIN_METERS &&
           point.getX() < FIELD_LENGTH_METERS - FIELD_MARGIN_METERS &&
           point.getY() > FIELD_MARGIN_METERS &&
           point.getY() < FIELD_WIDTH_METERS - FIELD_MARGIN_METERS;
  }
  
  // Sanity check for unanchored poses (SHOP_RESUME mode)
  // Rejects NaN/Inf and absurdly large values (>100m suggests Quest error)
  private boolean isSanePose(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double theta = pose.getRotation().getRadians();
    
    // Check for NaN/Inf
    if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(theta)) {
      Logger.recordOutput("PoseInitializer/InsanePose", "NaN/Inf detected");
      return false;
    }
    
    // Check for absurdly large values (Quest bug/corruption)
    double magnitude = Math.hypot(x, y);
    if (magnitude > MAX_SANE_POSE_MAGNITUDE) {
      Logger.recordOutput("PoseInitializer/InsanePose", 
          String.format("Magnitude too large: %.2fm", magnitude));
      return false;
    }
    
    return true;
  }
  
  public boolean isInitialized() {
    return initState == InitializationState.INITIALIZED || 
           initState == InitializationState.FALLBACK_USED;
  }
  
  public InitializationState getInitState() {
    return initState;
  }
  
  public void setInitState(InitializationState state) {
    this.initState = state;
  }
  
  public double getWaitTime() {
    return initWaitTimer.get();
  }
}
