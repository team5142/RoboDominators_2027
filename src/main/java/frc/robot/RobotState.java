package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MatchPhaseTracker;
import frc.robot.util.SmartLogger;

// Global robot state tracker - coordinates subsystem states and robot intents
// Single source of truth for robot mode, navigation phase, and robot intents
public class RobotState {

  // Game state (FMS-driven)
  public enum Mode {
    DISABLED,
    ENABLED_TELEOP,
    ENABLED_AUTO,
    TEST
  }

  private Mode mode = Mode.DISABLED;
  private boolean enabled = false;
  private boolean sysIdMode = false;
  private boolean operatorDriveLockout = false;
  // When true: QuestNav is unreliable and pose-dependent commands are blocked.
  private boolean questNavEmergencyMode = false;
  // When true: auto shoot mode is active and ball-handling runs automatically.
  private boolean autoShootMode = false;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

  // Match phase and hub active tracking
  private final MatchPhaseTracker matchPhaseTracker = new MatchPhaseTracker();
  // Navigation state (active - used by SmartDrive)
  public enum NavigationPhase {
    NONE,           // Not navigating
    FAST_APPROACH,  // PathPlanner pathfinding
    PRECISION_PATH, // AutoPilot precision
    LOCKED          // Navigation complete, wheels locked
  }

  private NavigationPhase navigationPhase = NavigationPhase.NONE;

  public enum ShootingZone {
    ALLIANCE,   // our side - shoot into hub when hub is active
    NEUTRAL,    // mid-field - pass back during first 15s of opponent period
    OPPONENT    // opponent side - never shoot
  }

  private ShootingZone shootingZone = ShootingZone.ALLIANCE;

  // Field position
  private Pose2d robotPose = new Pose2d();

  public void setNavigationPhase(NavigationPhase navPhase) {
    if (this.navigationPhase == navPhase) {
      return;
    }
    this.navigationPhase = navPhase;
    SmartLogger.logReplay("RobotState/NavigationPhase", navPhase.toString());
  }

  public NavigationPhase getNavigationPhase() { return navigationPhase; }

  public void setRobotPose(Pose2d pose) { this.robotPose = pose; }
  public Pose2d getRobotPose() { return robotPose; }

  public void setMode(Mode mode) {
    if (this.mode == mode) {
      return;
    }
    this.mode = mode;
    SmartLogger.logReplay("RobotState/Mode", mode.toString());
  }

  public Mode getMode() { return mode; }

  public void setAlliance(DriverStation.Alliance alliance) {
    if (this.alliance == alliance) {
      return;
    }
    this.alliance = alliance;
    SmartLogger.logReplay("RobotState/Alliance", alliance.toString());
  }

  public DriverStation.Alliance getAlliance() { return alliance; }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    SmartLogger.logReplay("RobotState/Enabled", enabled);
  }

  public boolean isEnabled() { return enabled; }

  public void setSysIdMode(boolean sysIdMode) {
    this.sysIdMode = sysIdMode;
    SmartLogger.logConsole(
        "SysId mode " + (sysIdMode ? "enabled - vision updates disabled" : "disabled"),
        "SysId Mode");
  }

  public boolean isSysIdMode() { return sysIdMode; }

  public boolean isQuestNavEmergencyMode() { return questNavEmergencyMode; }

  public void setQuestNavEmergencyMode(boolean active) {
    questNavEmergencyMode = active;
    SmartLogger.logConsole(
        "QuestNav emergency mode " + (active ? "ACTIVE - pose commands blocked" : "cleared"),
        "Emergency");
  }

  public boolean isAutoShootMode() { return autoShootMode; }

  public void setAutoShootMode(boolean active) {
    autoShootMode = active;
    SmartLogger.logConsole("Auto shoot mode " + (active ? "ACTIVE" : "OFF"), "AutoShoot");
  }

  public ShootingZone getShootingZone() { return shootingZone; }

  public void setShootingZone(ShootingZone zone) {
    if (shootingZone == zone) return;
    shootingZone = zone;
    SmartLogger.logReplay("RobotState/ShootingZone", zone.toString());
  }

  public boolean isOperatorDriveLockout() {
    return operatorDriveLockout;
  }

  public void setOperatorDriveLockout(boolean operatorDriveLockout) {
    if (this.operatorDriveLockout == operatorDriveLockout) {
      return;
    }
    this.operatorDriveLockout = operatorDriveLockout;
    SmartLogger.logReplay("RobotState/OperatorDriveLockout", operatorDriveLockout);
  }

  // Updates match phase state - call once per teleop periodic loop.
  public void updateMatchPhase() {
    matchPhaseTracker.update();
  }

  // True if our alliance hub is currently active for scoring.
  public boolean isHubActive() {
    return matchPhaseTracker.isHubActive();
  }

  // True if our hub will be active within leadSeconds from now.
  // Use to start flywheel spin-up before the window opens.
  public boolean isHubActiveIn(double leadSeconds) {
    return matchPhaseTracker.isHubActiveIn(leadSeconds);
  }

  // True if we should be shooting right now, accounting for lead/stop-early times.
  // leadSeconds: start spinning up this many seconds before a window opens.
  // stopEarlySeconds: stop feeding balls this many seconds before a window closes (flight time).
  public boolean shouldShoot(double leadSeconds, double stopEarlySeconds) {
    return matchPhaseTracker.shouldShoot(leadSeconds, stopEarlySeconds);
  }

  // Zone-aware shoot gate - checks match phase and current robot zone.
  // Returns true when shooting/passing is allowed. Always true in plain teleop (no match time).
  public boolean shouldShootInZone() {
    return matchPhaseTracker.shouldShootInZone(shootingZone);
  }

  public MatchPhaseTracker.GamePhase getGamePhase() {
    return matchPhaseTracker.getPhase();
  }

  public String getPhaseName()            { return matchPhaseTracker.getPhaseName(); }
  public int getShiftNumber()             { return matchPhaseTracker.getShiftNumber(); }
  public double getSecondsUntilPhaseEnd() { return matchPhaseTracker.getSecondsUntilPhaseEnd(); }
  public boolean hasMatchGameData()       { return matchPhaseTracker.hasGameData(); }
}
