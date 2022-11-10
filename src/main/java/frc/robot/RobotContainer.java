// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmIntake.ArmState;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(JoystickConstants.Driver.driverPort);
  private final Joystick secondary = new Joystick(JoystickConstants.Secondary.secondaryPort);

  /* Drive Controls */
  private final int translationAxis = driver.getYChannel();
  private final int strafeAxis = driver.getXChannel();
  private final int rotationAxis = driver.getZChannel();

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, JoystickConstants.Driver.zeroGyro);
  private final JoystickButton defenseButton = new JoystickButton(driver, JoystickConstants.Driver.driverPort);

  /* Secondardy Buttons */
  private final JoystickButton lowerArm = new JoystickButton(secondary, JoystickConstants.Secondary.intakeAndArm);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ArmIntake s_ArmIntake = new ArmIntake();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

    defenseButton.whenPressed(new DefenseModeCommand(s_Swerve, true));
    defenseButton.whenReleased(new DefenseModeCommand(s_Swerve, false));

    lowerArm.whenPressed(new SetArmState(s_ArmIntake, ArmState.LOW));
    lowerArm.whenPressed(new SetArmState(s_ArmIntake, ArmState.HIGH));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}