// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSNeoSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CANIDConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kFrontLeftTurningCanId = 11;
    public static final int kFrontLeftCanCoderCanId = 12;
    public static final double kFrontLeftTurningOffset = 231.5;

    public static final int kFrontRightDrivingCanId = 20;
    public static final int kFrontRightTurningCanId = 21;
    public static final int kFrontRightCanCoderCanId = 22;
    public static final double kFrontRightTurningOffset  = 317;

    public static final int kRearRightDrivingCanId = 30;
    public static final int kRearRightTurningCanId = 31;
    public static final int kRearRightCanCoderCanId = 32;
    public static final double kRearRightTurningOffset  = 253.7;

    public static final int kRearLeftDrivingCanId = 40;
    public static final int kRearLeftTurningCanId = 41;
    public static final int kRearLeftCanCoderCanId = 42;
    public static final double kRearLeftTuriningOffsset  = 181.1;

    // Power Distribution Hub (PDH) CAN ID
    public static final int kPdhCanId = 1;

    public static final int kPigeon2ID = 4;
  }

  public class IDConstants {

    public static final int FRONT_LEFT_LOCATION = 0;
    public static final int FRONT_RIGHT_LOCATION = 1;
    public static final int REAR_LEFT_LOCATION = 2;
    public static final int REAR_RIGHT_LOCATION = 3;

  }

  public static final class PPConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kMaxAngularSpeed
        / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kPXController = 2;
    public static final double kDXController = 0;
    public static final double kIXController = 0;

    public static final double kPYController = 2;
    public static final double kDYController = 0;
    public static final double kIYController = 0;

    public static final double kPThetaController = 0.1;
    public static final double kDThetaController = 0;
    public static final double kIThetaController = 0;

  }

  public class PDPConstants {

    public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
    public static final int FRONT_RIGHT_DRIVE_CHANNEL = 1;
    public static final int BACK_LEFT_DRIVE_CHANNEL = 1;
    public static final int BACK_RIGHT_DRIVE_CHANNEL = 1;

    public static final int FRONT_LEFT_TURN_CHANNEL = 1;
    public static final int FRONT_RIGHT_TURN_CHANNEL = 1;
    public static final int BACK_LEFT_TURN_CHANNEL = 1;
    public static final int BACK_RIGHT_TURN_CHANNEL = 1;

  }

  public static final class DriverConstants {

    public static double kTranslationSlew = 10.0;
    public static double kRotationSlew = 10.0;
    public static double kControllerDeadband = .05;
    public static double kControllerRotDeadband = .1;
    public static final double stickDeadband = 0.1;

  }
  public static final class Swerve {
    public static final int pigeonID = 4;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSNeoSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
    COTSNeoSwerveConstants.SDSMK4i(COTSNeoSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final IdleMode  angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = CANIDConstants.kFrontLeftDrivingCanId;
        public static final int angleMotorID = CANIDConstants.kFrontLeftTurningCanId;
        public static final int canCoderID = CANIDConstants.kFrontLeftCanCoderCanId;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = CANIDConstants.kFrontRightDrivingCanId;
        public static final int angleMotorID = CANIDConstants.kFrontRightTurningCanId;
        public static final int canCoderID = CANIDConstants.kFrontRightCanCoderCanId;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = CANIDConstants.kRearLeftDrivingCanId;
        public static final int angleMotorID = CANIDConstants.kRearLeftTurningCanId;
        public static final int canCoderID = CANIDConstants.kRearLeftCanCoderCanId;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = CANIDConstants.kRearRightDrivingCanId;
        public static final int angleMotorID = CANIDConstants.kRearRightTurningCanId;
        public static final int canCoderID = CANIDConstants.kRearRightCanCoderCanId;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}
  public static final class DriveConstants {
    public static final double kMaxRotationRadiansPerSecond = Math.PI;

    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI;
  
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.46;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 3.6; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot

// v  public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
//   m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kBackLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final boolean kGyroReversed = false;

    // Drivetrain Speeds
    public static final double kDriveMaxOutput = 0.95; // Field: 0.85 | Arcadia: 0.40
    public static final double kDriveSlow = 0.25; // 0.25
    public static final double kmaxPOVturnspeed = 1.0; // 0.45
    public static final double kAutoLevelMaxOutput = 0.30;
    // Vision
    public static final double maxVisionRotSpeed = 0.4;
    public static final double maxVisionStrafeSpeed = 1.0; // handled by maxOutput (TODO: Add xExeption param to
                                                           // drive(...))
    public static final double kRobotHeadingTolerance = 1.0; // in degrees

    // Cardinal Directions
    public static final double faceForward = 0;
    public static final double faceBackward = 180;
    public static final double faceLeft = 90;
    public static final double faceRight = -90;

    private final static Translation2d m_frontLeftLocation = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_frontRightLocation = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    private final static Translation2d m_backLeftLocation = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    private final static Translation2d m_backRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);


    public static final Translation2d[] kModuleTranslations = {

      m_frontLeftLocation,
      m_frontRightLocation,
      m_backLeftLocation,
      m_backRightLocation };
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    // public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static double kVoltCompensation = 12.6;

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14 .122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static final double kDriveMetersPerEncRev =

        (kWheelDiameterMeters * Math.PI) / mk4iL1DriveGearRatio;// 0.039198257811106

    public static double kEncoderRevsPerMeter = 1 / kDriveMetersPerEncRev;// 25.511337897182322

    public static final double kTurningDegreesPerEncRev =

        360 / mk4iL1TurnGearRatio;

  }

  // Operator Input
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final int kTestControllerPort = 2;
    public static final double kDriveDeadband = 0.05;
    public static final double kJoystickDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kLevelTolerance = 2.25; // field tolerance is 2.25 degrees

    public static final double kDriveAngle = -11; // Was 14.0

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  }

  public static final class PathConstants {
    public static final double kpXdefault = 1.5;
    public static final double kiXdefault = 0.0;
    public static final double kdXdefault = 0.0;

    public static final double kpYdefault = 3.0;
    public static final double kiYdefault = 0.0;
    public static final double kdYdefault = 0.0;

    public static final double kpRdefault = 5.0;
    public static final double kiRdefault = 0.0;
    public static final double kdRdefault = 0.0;

    public static final int LVL = 0;
    public static final int NoLVL = 1;
    public static final int grabCube = 2;

  }

  public static final class VisionConstants {
    public static final double kLimelightOffDelay = 3.0;
    public static final String tLength = "thor";
    public static final double kDeltaThreshhold = 4; // in degrees
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class CurrentLimitConstants {

    public static final int turnMotorSmartLimit = 20;

    public static final int driveMotorSmartLimit = 20;

  }

  public final static class ModuleTuneConstants {

    public static final double kPModuleDriveController = .2;
    public static final double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0;

    public static final double kPModuleTurningController = .004;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;

  }

  public static final class SYSIDConstants {
    // from Beta test
    public static final double ksDriveVoltSecondsPerMeter = .0927;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 3.13;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.82;
    // sysid on module?
    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3

  }
}
