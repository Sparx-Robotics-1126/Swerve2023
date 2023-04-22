// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAngle extends PIDCommand {
  /** Creates a new RotateToAngle. */


  public RotateToAngle(DriveSubsystem drive, double angle) {
    super(
        // The controller that the command will use
        new PIDController(0.02, 0, 0),
        // This should return the measurement
        () -> drive.getHeadingDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drive.drive(0, 0, output);
        });

        super.m_controller.setTolerance(1);
        super.m_controller.enableContinuousInput(0, 360);
        //this number could be changed

        addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint();
  }
}