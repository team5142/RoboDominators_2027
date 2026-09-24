package frc.robot.route;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.route.RouteConstraints;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.QuestNavSubsystem;

public class BLineCoarseRouteProvider implements CoarseRouteProvider{
    public DriveSubsystem driveSubsystem;
    public ChassisSpeeds robotSpeed;
    public QuestNavSubsystem questNavSubsystem;
    public Pose2d currentPose;
    public RobotState robotState;
    
    public Command createRouteCommand(RouteRequest request) {
        Path goalPath=new Path( 
            new Path.Waypoint(request.targetPose())
        );
        
        FollowPath.Builder pathBuilder = new FollowPath.Builder(
            driveSubsystem,
            driveSubsystem::getRobotPose,
            driveSubsystem::getRobotRelativeSpeeds,
            driveSubsystem::driveRobotRelative,
            new PIDController(5.0, 0, 0),  // translation
            new PIDController(3.0, 0, 0),  // rotation
            new PIDController(2.0, 0, 0)   // cross-track
            ).withDefaultShouldFlip();
            return pathBuilder.build(goalPath);

        
        
            
            
            
            
    }

}
    
  

