package frc.robot.route;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import choreo.auto.*;
import choreo.trajectory.*;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.PrintCommand;
public class ChoreoCoarseRouteProvider implements CoarseRouteProvider  {
    public DriveSubsystem driveSubsystem;
    public ChassisSpeeds relativeSpeeds;
    public AutoFactory autoFactory;
    public Command myTrajectory;
    public ChoreoCoarseRouteProvider() {
        this.driveSubsystem=driveSubsystem;
        autoFactory = new AutoFactory(
            driveSubsystem::getRobotPose,
            driveSubsystem::resetPose,
            driveSubsystem::driveRobotRelative, 
            true,
             driveSubsystem);
        myTrajectory=autoFactory.trajectoryCmd("myTrajectory");
    }
        public Command createRouteCommand(RouteRequest request) {
                if (myTrajectory!=null) {
                    return myTrajectory;
                } 
                else{ 
                    return new PrintCommand("no trajectory available");
                }
            }
                                


    
    
    

}
