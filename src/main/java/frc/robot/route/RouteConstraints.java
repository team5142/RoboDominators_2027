package frc.robot.route;

// Speed/acceleration limits for a coarse route. Team-owned so any CoarseRouteProvider
// implementation (PathPlanner, BLine, ...) can be given the same limits.
public record RouteConstraints(
    double maxVelocityMps,
    double maxAccelerationMps2,
    double maxAngularVelocityRadPerSec,
    double maxAngularAccelerationRadPerSec2) {}
