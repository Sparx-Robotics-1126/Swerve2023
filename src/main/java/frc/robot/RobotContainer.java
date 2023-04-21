// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;

// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.PathPlanner.C1ThreePiece;
// import frc.robot.commands.PathPlanner.C1TwoPiece;
// import frc.robot.commands.PathPlanner.C3OneCone;
// import frc.robot.commands.PathPlanner.C3OneCubeHybrid;
// import frc.robot.commands.PathPlanner.C5OneCubeLevel;
// import frc.robot.commands.PathPlanner.C7HybridLink;
// import frc.robot.commands.PathPlanner.C4C6OneConeLevel;
// import frc.robot.commands.PathPlanner.C9TwoPiece;
// import frc.robot.commands.PathPlanner.OneConeScoreSolo;
// import frc.robot.commands.PathPlanner.OneCubeScoreSolo;
// import frc.robot.commands.PathPlanner.C7OneCone;
// import frc.robot.commands.PathPlanner.C9OneConeShootTwo;
// import frc.robot.commands.armCommands.ArmToPosition;
// import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.commands.driveCommands.AutoLevel;
import frc.robot.commands.driveCommands.DriveToLevel;
// import frc.robot.commands.groundIntakeCommands.IntakeHandoff;
import frc.robot.commands.navXCommands.ResetGyro;
import frc.robot.subsystems.DriveSubsystem.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  // private final UsbCamera m_cameraUSB;

  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_manipulatorController = 
      new CommandXboxController(OIConstants.kManipulatorControllerPort); 
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_testController =
      new CommandXboxController(OIConstants.kTestControllerPort);
      
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems
    m_robotDrive = new DriveSubsystem(m_driverController.rightBumper()); //Be careful when pressing this buttton while doing an auto command

    // Configure the button bindings
    configureButtonBindings();

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // 0 is default, 1 is camMode

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true, true,
                Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {


    // DRIVER 1

    // Zero Heading
    m_driverController.leftStick().onTrue(new ResetGyro(m_robotDrive));

    // Prevents Movement
    m_driverController.povDown().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Charge Station Auto Level
    m_driverController.povUp().whileTrue(new DriveToLevel(m_robotDrive)); // Auto Level while ON Charge Station
    m_driverController.povLeft().whileTrue(new AutoLevel(m_robotDrive));  // Full AutoLevel Sequence while OFF Charge Station

    // Main Intake Drop-off
    // Intake deadband to prevent accidental activation
    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> 
      m_driverController.getRightTriggerAxis()));

    // POV Rotation
    m_driverController.b().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceRight,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.x().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceLeft,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.y().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceForward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));
    m_driverController.a().whileTrue( new RunCommand (
          () -> m_robotDrive.TurnToTarget(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              Constants.DriveConstants.faceBackward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));

    // Toggle for field oriented vs robot oriented
    // When right stick pressed down, run the robot oriented drive.
    // When right stick pressed down again, end the robot oriented drive and run default drive, which is field oriented drive
    // NOT USED IN MATCH PLAY
    m_driverController.rightStick().toggleOnTrue( new RunCommand (
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              false, true, true,
              Constants.DriveConstants.kDriveMaxOutput),
          m_robotDrive));

      // Allign with Cone Node by Rotation
      m_driverController.start().whileTrue(new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              m_robotDrive.getVisionRotSpeed(),
              true, true, true,
              Constants.DriveConstants.kDriveMaxOutput,
              true),
            m_robotDrive)
              .beforeStarting(new SequentialCommandGroup(
                new RunCommand(() -> m_robotDrive.setLimelightLEDsOn()).withTimeout(0.1), 
                new InstantCommand(() -> m_robotDrive.setVisionOriginaltx())))
              .handleInterrupt(() -> m_robotDrive.setLimelightLEDsOff()
      ));

      // Allign with Cone Node by Strafe
      m_driverController.back().whileTrue(new RunCommand(
          () -> m_robotDrive.TurnToTarget(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -m_robotDrive.getVisionStrafeSpeed(),
              Constants.DriveConstants.faceBackward,
              true, true,
              Constants.DriveConstants.kDriveMaxOutput),
            m_robotDrive)
              .beforeStarting(new InstantCommand(() -> m_robotDrive.setLimelightLEDsOn()))
      );



    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
