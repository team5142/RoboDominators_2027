package frc.robot.route;

import edu.wpi.first.math.geometry.Pose2d;

// A request to drive coarsely to a field pose, handed to a CoarseRouteProvider.
public record RouteRequest(Pose2d targetPose, RouteConstraints constraints) {}
