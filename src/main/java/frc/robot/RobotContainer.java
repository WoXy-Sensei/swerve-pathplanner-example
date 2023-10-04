// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Swerve m_Swerve = new Swerve();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_Swerve.setDefaultCommand(
        new TeleopSwerve(
            m_Swerve,
            () -> MathUtil.applyDeadband(m_driverController.getLeftY(), SwerveConstants.stickDeadband), // translationAxis
            () -> MathUtil.applyDeadband(m_driverController.getLeftX(), SwerveConstants.stickDeadband), // strafeAxis
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), SwerveConstants.stickDeadband), // rotationAxis
            () -> m_driverController.leftBumper().getAsBoolean()));

    configureAutonomous();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.start().onTrue(Commands.runOnce(() -> m_Swerve.zeroGyro()));
  };

  public void configureAutonomous() {
    HashMap<String, Command> autonomous = new HashMap<>();
    autonomous.put("test1", new test(m_Swerve));

    /* set in dashboard */
    m_chooser.setDefaultOption("NULL", null);
    autonomous.forEach((commandName, command) -> m_chooser.addOption(commandName, command));

    Shuffleboard.getTab("Auto").add(m_chooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
