package frc.robot.commands.PathPlanner.Tests;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
// import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.commands.driveCommands.DriveToLevel;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

public class OneConeAuto1 extends SequentialCommandGroup {

    public OneConeAuto1(DriveSubsystem driveSubsystem ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 1", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 1", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave Community 1",
            new PathConstraints(2, 3));

        
        addCommands(
           
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
       
            new ParallelCommandGroup(
            driveSubsystem.followTrajectoryCommand(m_thirdPath, false)),
            // new AutoLevel(driveSubsystem)
            new DriveToLevel(driveSubsystem)
            );
    }
}
