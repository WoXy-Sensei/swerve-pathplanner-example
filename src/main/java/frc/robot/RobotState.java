// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class RobotState {

    private static RobotState robotState;
    public enum GamePiece{
        CONE,
        CUBE, 
        EMPTY
    }
    public enum ElevatorLevel{
        ZERO,
        CARRY,
        MIDCUBE,
        MIDCONE,
        TOPCUBE,
        TOPCONE,
        DEFAULT
    }
    public enum GripperState{
        INTAKINGCUBE,
        INTAKINGCONE,
        RELEASINGCONE,
        THROWINGCUBE,
        INTAKED,
        STOP

    }
    public enum GripperPistonState{
        FORWARD,
        REVERSE,
        STOP
    }

    public enum AutoState{
        NORMAL,
        BALANCING,
        BALANCED,
        NOAUTO
    }

    //public enum CarriageState{
    //    AUTO,
    //    MANUAL,
   // }

    public enum SwerveState{
        MOVING,
        REST
    }

    public static GripperPistonState currentGripperPistonState = GripperPistonState.STOP;
    public static GripperState currentGripperState = GripperState.STOP;
    public static GamePiece currentGamePiece = GamePiece.EMPTY;
    public static ElevatorLevel currentElevatorLevel = ElevatorLevel.ZERO;
    public static SwerveState currentSwerveState = SwerveState.REST;
    public static AutoState currentAutoState = AutoState.NOAUTO;
    public static double balanceGyroAngle=0;

//    public static CarriageState currentCarriageState = CarriageState.AUTO;
    public static boolean isElevated = false;
    public static boolean isTripping = false;
    public static boolean isGripperReverse = true;
    public static boolean isIMUalive = false;
    
    private RobotState(){
        //reset(); 
    }

    public static void setShooting(){
        //currentIntakeState = GripperState.SHOOTING;
    }

    //public static boolean isShooting(){
        //return currentIntakeState == GripperState.SHOOTING;
    //}

    public static boolean isTeleop(){
        return edu.wpi.first.wpilibj.RobotState.isTeleop();
    }

    public static boolean isBlueAlliance(){
        return DriverStation.getAlliance() == Alliance.Blue;
    }

    public static boolean isRedAlliance(){
        return DriverStation.getAlliance() == Alliance.Red;
    }

       
    public static void setGamePiece(GamePiece gamePiece){
        if(currentGamePiece != gamePiece) currentGamePiece = gamePiece;
    }

    public static void setCone(){
        if(currentGamePiece != GamePiece.CONE) currentGamePiece = GamePiece.CONE;
    }

    public static void setCube(){
        if(currentGamePiece != GamePiece.CUBE) currentGamePiece = GamePiece.CUBE;        
    }

 
    public static void setElevated(boolean elevated){
        if(isElevated != elevated) isElevated = elevated;
    }

    public static void setSwerve(SwerveState swerveState){
        currentSwerveState = swerveState;
    }


    public static boolean isElevated(){
        return isElevated;
    }

 

    public static void reset(){
        //isCarriageHome = false;
        currentGamePiece = null;
        currentElevatorLevel = null;
        currentGripperState = GripperState.STOP;
        isElevated = false;
        isTripping = false;
    }

    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }



}
