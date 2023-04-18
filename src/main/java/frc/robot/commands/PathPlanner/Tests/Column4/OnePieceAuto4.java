package frc.robot.commands.PathPlanner.Tests.Column4;

// TODO: NOT TESTED

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
// import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;
// import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem;
// import frc.robot.subsystems.MainIntakeSubsystem.IntakeSubsystem;
// import frc.robot.subsystems.MainIntakeSubsystem.ArmSubsystem.armPositions;

public class OnePieceAuto4 extends SequentialCommandGroup {

    public OnePieceAuto4(DriveSubsystem driveSubsystem ){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 4", 
            new PathConstraints(2, 3));
        PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 4", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Leave Community 4", 
            new PathConstraints(2, 3));

        
        addCommands(
           
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
          
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
        
            driveSubsystem.followTrajectoryCommand(m_thirdPath, false),
            new AutoLevel(driveSubsystem)
            );
    }
}
