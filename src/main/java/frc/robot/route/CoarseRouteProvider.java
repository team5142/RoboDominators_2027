package frc.robot.route;

import edu.wpi.first.wpilibj2.command.Command;

// Vendor-neutral seam for coarse (non-precision) routing to a target pose.
// Lets SmartDriveToPosition's fast-approach phase be driven by PathPlanner today
// and by another route provider (e.g. BLine) later, without changing its own logic.
public interface CoarseRouteProvider {

  // Builds a command that coarsely drives to request.targetPose() within request.constraints().
  Command createRouteCommand(RouteRequest request);
}
