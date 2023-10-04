package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.utils.CANCoderUtil;
import frc.lib.utils.CANSparkMaxUtil;
import frc.lib.utils.CANCoderUtil.CCUsage;
import frc.lib.utils.CANSparkMaxUtil.Usage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;
 

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;
 

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

  

   /* Angle Encoder Config */
   angleEncoder = new CANCoder(moduleConstants.cancoderID);
   configAngleEncoder();

   /* Angle Motor Config */
   angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
   integratedAngleEncoder = angleMotor.getEncoder();
   angleController = angleMotor.getPIDController();
   configAngleMotor();

   /* Drive Motor Config */
   driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
   driveEncoder = driveMotor.getEncoder();
   driveController = driveMotor.getPIDController();
   configDriveMotor();

   lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    
    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = SwerveConstants.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    angleEncoder.configAllSettings(swerveCanCoderConfig);
  }
  
  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(SwerveConstants.angleInvert);
    angleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
    angleController.setP(SwerveConstants.angleKP);
    angleController.setI(SwerveConstants.angleKI);
    angleController.setD(SwerveConstants.angleKD);
    angleController.setFF(SwerveConstants.angleKFF);
    angleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(SwerveConstants.driveInvert);
    driveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
    driveController.setP(SwerveConstants.angleKP);
    driveController.setI(SwerveConstants.angleKI);
    driveController.setD(SwerveConstants.angleKD);
    driveController.setFF(SwerveConstants.angleKFF);
    driveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
     // Prevent rotating module if speed is less then 1%. Prevents jittering.
     Rotation2d angle =
     (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
         ? lastAngle
         : desiredState.angle;

      angleController.setReference(angle.getDegrees(), ControlType.kPosition);
      lastAngle = angle;
  }
  public void stopMotors(){
    driveMotor.set(0);
    angleMotor.set(0);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(),  getAngle());
        //Conversions.neoToMeters(driveEncoder.getPosition(), SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),      
    
}
}
