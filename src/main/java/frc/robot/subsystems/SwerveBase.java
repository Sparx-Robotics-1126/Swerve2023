package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.Robot;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ModuleTuneConstants;
import frc.robot.Constants.SYSIDConstants;
import frc.robot.utils.AngleUtils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveBase {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final CANSparkMax mAngleMotor;
    private final CANSparkMax mDriveMotor;

    public  RelativeEncoder m_driveEncoder;

    private  RelativeEncoder m_turnEncoder;


    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        SYSIDConstants.ksDriveVoltSecondsPerMeter,
        SYSIDConstants.kvDriveVoltSecondsSquaredPerMeter,
        SYSIDConstants.kaDriveVoltSecondsSquaredPerMeter);

        private double m_lastAngle;
        public double angle;

        public double m_turnEncoderOffset;

        private final PIDController m_driveVelController = new PIDController(ModuleTuneConstants.kPModuleDriveController,
        ModuleTuneConstants.kIModuleDriveController, ModuleTuneConstants.kDModuleDriveController);
  
    private PIDController m_turnPosController = new PIDController(ModuleTuneConstants.kPModuleTurningController,
        ModuleTuneConstants.kIModuleTurningController, ModuleTuneConstants.kDModuleTurningController);
  

        private int tst;

    public SwerveBase(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 


        SmartDashboard.putNumber("StateSpeed", desiredState.speedMetersPerSecond);

        SmartDashboard.putNumber("StateDegs", desiredState.angle.getDegrees());
    
        // turn motor code
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
        if ((Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeedMetersPerSecond * 0.01)))
    
          angle = m_lastAngle;
    
        else
    
          angle = desiredState.angle.getDegrees();
    
        m_lastAngle = angle;

        driveMotorMove(desiredState.speedMetersPerSecond, isOpenLoop);

        positionTurn(angle);



        // setAngle(desiredState);
        // setSpeed(desiredState, isOpenLoop);
    }
    public void driveMotorMove(double speed, boolean isOpenLoop) {

        if (isOpenLoop) {
            mDriveMotor
              .setVoltage(RobotController.getBatteryVoltage() * speed / DriveConstants.kMaxSpeedMetersPerSecond);
    
          SmartDashboard.putNumber("DrFF Volts", feedforward.calculate(speed));
    
          // m_driveMotor.setVoltage(feedforward.calculate(speed));
    
        } else {
    
            mDriveMotor.setVoltage(feedforward.calculate(speed)
              + m_driveVelController.calculate(getDriveVelocity(), speed));
    
        }
      }
    
      public void positionTurn(double angle) {
    
        double pidOut = m_turnPosController.calculate(getTurnAngleDegs(), angle);
    
        SmartDashboard.putNumber("PIDOUT", pidOut);
    
        double turnAngleError = Math.abs(angle - getTurnAngleDegs());
    
        SmartDashboard.putNumber("ATAE", turnAngleError);
    
        SmartDashboard.putNumber("TEST", tst++);
    
        // if robot is not moving, stop the turn motor oscillating
        // if (turnAngleError < turnDeadband
    
        // && Math.abs(state.speedMetersPerSecond) <=
        // (DriveConstants.kMaxSpeedMetersPerSecond * 0.01))
    
        // pidOut = 0;
    
        mAngleMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    
      }
    // private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    //     if(isOpenLoop){

    //         double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
    //         // mDriveMotor.setOpenLoopRampRate(1);
    //         // mDriveMotor.set(percentOutput);

    //         mDriveMotor
    //         .setVoltage(RobotController.getBatteryVoltage() * percentOutput / DriveConstants.kMaxSpeedMetersPerSecond);
    //     }
    //     else {
    //         // double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

    //         mDriveMotor.setVoltage(feedforward.calculate(desiredState.speedMetersPerSecond)
    //         + m_driveVelController.calculate(getDriveVelocity(), speed));

    //         mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    //     }
    // }

    // private void setAngle(SwerveModuleState desiredState){
    //     Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
    //     mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
    //     lastAngle = angle;
    // }

    // private Rotation2d getAngle(){
    //     return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    // }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor.setSelectedSensorPosition(absolutePosition);

        double angle = 0;
        if (RobotBase.isReal())
          angle = angleEncoder.getAbsolutePosition() - angleOffset.getDegrees();
        m_turnEncoder.setPosition(angle);
    }

    private void configAngleEncoder(){  
        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(AngleUtils.generateCanCoderConfig());
     
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){

        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setSmartCurrentLimit(CurrentLimitConstants.turnMotorSmartLimit);
        mAngleMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);

        mAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        mAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        mAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);
        // Set neutral mode to brake
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);

        m_turnEncoder = mAngleMotor.getEncoder();

        m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);
    
        m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);
    


        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();

        mDriveMotor.setSmartCurrentLimit(CurrentLimitConstants.driveMotorSmartLimit);

        mDriveMotor.enableVoltageCompensation(ModuleConstants.kVoltCompensation);

        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        mDriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        mDriveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Set neutral mode to brake
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);

        m_driveEncoder = mDriveMotor.getEncoder();

        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
    
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerEncRev
            / 60);

        m_turnPosController.enableContinuousInput(-180, 180);
        // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        // mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
    }

    public double getDriveVelocity() {
        if (RobotBase.isReal())
          return m_driveEncoder.getVelocity();
        else
          return (m_driveEncoder.getVelocity() * ModuleConstants.kDriveMetersPerEncRev) / 60;
    
      }
      public double getTurnAngleDegs() {
        if (RobotBase.isReal())
          return m_turnEncoder.getPosition();
        else
          return angle;
        // return m_turnEncoder.getPosition() *
        // ModuleConstants.kTurningDegreesPerEncRev;
    
      }
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees((getTurnAngleDegs())));
        // return new SwerveModuleState(
        //     Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
        //     getAngle()
        // ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getHeadingRotation2d());
        // return new SwerveModulePosition(
        //     Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
        //     getAngle()
        // );
    }

    public double getHeadingDegs() {
        return getTurnAngleDegs();
      }
    
      public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegs());
      }
    
    public double getDrivePosition() {
        if (RobotBase.isReal())
          return m_driveEncoder.getPosition();
        else
          return m_driveEncoder.getPosition() * ModuleConstants.kDriveMetersPerEncRev;
    
      }

      public void tunePosGains() {
        m_turnPosController.setP(Pref.getPref("SwerveTurnPoskP"));
        m_turnPosController.setI(Pref.getPref("SwerveTurnPoskI"));
        m_turnPosController.setD(Pref.getPref("SwerveTurnPoskD"));
        // m_turnController.setIZone(Pref.getPref("SwerveTurnPoskIz"));
      }
    
      public void tuneDriveVelGains() {
        m_driveVelController.setP(Pref.getPref("SwerveVelkP"));
        m_driveVelController.setI(Pref.getPref("SwerveVelkI"));
        m_driveVelController.setD(Pref.getPref("SwerveVelkD"));
    
      }
}