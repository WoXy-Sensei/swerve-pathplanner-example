package frc.robot.subsystems;


import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.SwerveState;

public class Swerve extends SubsystemBase {

  AHRS ahrs;

 
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
   
    try {
		
      ahrs = new AHRS(SPI.Port.kMXP);       
    
    } catch (RuntimeException ex ) {
     
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, SwerveConstants.Mod0.constants),
          new SwerveModule(1, SwerveConstants.Mod1.constants),
          new SwerveModule(2, SwerveConstants.Mod2.constants),
          new SwerveModule(3, SwerveConstants.Mod3.constants)
        };
   
    Timer.delay(1.0);
    resetModulesToAbsolute();
    
    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYawNavX(), getModulePositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if(translation.getX() < 0.1 && translation.getY() < 0.1 && rotation < 0.2) 
      RobotState.setSwerve(SwerveState.REST);
    else 
      RobotState.setSwerve(SwerveState.MOVING);
    
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYawNavX())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  
  public void goStraight(Translation2d translation, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(translation.getX(), translation.getY(), 0.0));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  public SwerveAutoBuilder getAutoBuilder(Map<String, Command> eventMap) {
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      this::getPose, // Pose2d supplier
      this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      SwerveConstants.swerveKinematics, // SwerveDriveKinematics
      AutoConstants.CONSTANTS_X, // PID constants to correct for translation error (used to create the X and Y PID controllers)
      AutoConstants.THETA_CONSTANTS, // PID constants to correct for rotation error (used to create the rotation controller)
      this::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      this // The drive subsystem. Used to properly set the requirements of path following commands
    );  

    return autoBuilder;
    
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }
  
  public void resetOdometry(Pose2d pose) {    
    swerveOdometry.resetPosition(getYawNavX(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public void zeroGyro() { 
    ahrs.reset();
  }
  public void stopModules(){
    for(SwerveModule mod : mSwerveMods){
      mod.stopMotors();
  }
  }

  public double getHeadingNavX() {
    return Math.IEEEremainder(ahrs.getAngle(),360)*(SwerveConstants.invertGyro ? -1.0 : 1.0);
  }
  public Rotation2d getYawNavX(){
    return (SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(getHeadingNavX()) 
        : Rotation2d.fromDegrees(360-getHeadingNavX());
  }
 
  public void resetModulesToAbsolute(){
    for(SwerveModule mod : mSwerveMods){
        mod.resetToAbsolute();
    }
}


 

  @Override
  public void periodic() {    

    swerveOdometry.update(getYawNavX(), getModulePositions()); 
    field.setRobotPose(getPose());
    

    SmartDashboard.putNumber("NavxGetyaw", getYawNavX().getDegrees());
  
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + "PoseMeter", mod.getPosition().distanceMeters);
    }
    
  }
}