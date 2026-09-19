package frc.robot.route;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;

// CoarseRouteProvider backed by PathPlanner's dynamic pathfinder.
// Relies on AutoBuilder already being configured (RobotContainer.configurePathPlanner())
// before createRouteCommand() is called - true as soon as the robot boots, same as today.
public class PathPlannerCoarseRouteProvider implements CoarseRouteProvider {

  @Override
  public Command createRouteCommand(RouteRequest request) {
    PathConstraints constraints = new PathConstraints(
        request.constraints().maxVelocityMps(),
        request.constraints().maxAccelerationMps2(),
        request.constraints().maxAngularVelocityRadPerSec(),
        request.constraints().maxAngularAccelerationRadPerSec2());
    return AutoBuilder.pathfindToPose(request.targetPose(), constraints);
  }
}
