package frc.robot.route;

// Selects which CoarseRouteProvider implementation SmartDriveToPosition's fast-approach
// phase uses. PathPlanner is the committed default; BLine is a narrow, evaluation-only
// alternative (see BLineCoarseRouteProvider for its obstacle-avoidance limitation).
public enum CoarseRouteProviderType {
  PATHPLANNER,
  BLINE
}
