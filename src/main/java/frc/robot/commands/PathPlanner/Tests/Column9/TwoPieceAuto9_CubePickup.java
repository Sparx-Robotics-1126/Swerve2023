package frc.robot.commands.PathPlanner.Tests.Column9;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

public class TwoPieceAuto9_CubePickup extends SequentialCommandGroup {

    public TwoPieceAuto9_CubePickup(DriveSubsystem driveSubsystem){

        PathPlannerTrajectory m_firstPath = PathPlanner.loadPath("Cone Score 9", 
            new PathConstraints(2, 3));
            PathPlannerTrajectory m_secondPath = PathPlanner.loadPath("Cone Reverse 9", 
            new PathConstraints(2, 1));
        PathPlannerTrajectory m_thirdPath = PathPlanner.loadPath("Full Drive to Cube 9", 
            new PathConstraints(2, 2));
        // PathPlannerTrajectory m_forthPath = PathPlanner.loadPath("Full Drive to Cube2 9", 
        //     new PathConstraints(2, 2));
// forth path needs review/testing
        
        // PathPlannerTrajectory m_fifthPath = PathPlanner.loadPath("New Drive to Cube 9", 
        //     new PathConstraints(2, 3));
        // PathPlannerTrajectory m_sixthPath = PathPlanner.loadPath("Go to 8 with Cube", 
        //     new PathConstraints(1, 2));

        
        addCommands(
          
            driveSubsystem.followTrajectoryCommand(m_firstPath, true),
        
            driveSubsystem.followTrajectoryCommand(m_secondPath, false),
            new ParallelCommandGroup(
                driveSubsystem.followTrajectoryCommand(m_thirdPath, false))
            // driveSubsystem.followTrajectoryCommand(m_forthPath, false),
            // driveSubsystem.followTrajectoryCommand(m_fifthPath, false)
            // new DriveToLevel(driveSubsystem)
            );
    }
}
