package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.Constants.QuestNav;
import frc.robot.lib.BLine.BLineCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class BlineAutos {
        public static SendableChooser<Command> chooser= new SendableChooser<>();
        public static Path doNothing;
        public DriveSubsystem driveSubsystem;
        public PoseEstimatorSubsystem poseEstimator;
        public QuestNavSubsystem questNavSubsystem;
        public GyroSubsystem gyroSubsystem;
        public static Path samplePath;
        
    public static SendableChooser<Command> AutoChoices(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator) {
        doNothing = new Path ("doNothing"); // create default nothing path
        samplePath= new Path ("SamplePath"); // create placeholder path
        FollowPath.Builder bLineBuilder = new FollowPath.Builder(
            driveSubsystem,
            poseEstimator::getEstimatedPose,
            driveSubsystem::getRobotRelativeSpeeds,
            driveSubsystem::driveRobotRelative,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0)); // create the global builder for each path
        chooser.setDefaultOption("doNothing", bLineBuilder.build(doNothing));
        chooser.addOption("samplePath",bLineBuilder.build(samplePath));
        return chooser;
    }
}
