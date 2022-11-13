// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.autos.TaxiAuto;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.SetIntakeArmState;
import frc.robot.commands.ProgressClimberArmState;
import frc.robot.commands.SetClimberElevatorState;
import frc.robot.commands.SetRollerState;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ElevatorState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.ArmState;
import frc.robot.subsystems.Intake.RollerState;


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
  private final JoystickButton defenseButton = new JoystickButton(driver, JoystickConstants.Driver.defenseMode);

  /* Secondardy Buttons */
  private final JoystickButton armButton = new JoystickButton(secondary, JoystickConstants.Secondary.intakeAndArm);
  private final JoystickButton shootButton = new JoystickButton(secondary, JoystickConstants.Secondary.shoot);

  private final JoystickButton lowerArmNoSpinButton = new JoystickButton(secondary, JoystickConstants.Secondary.lowerArmNoSpin);
  private final JoystickButton forceRaiseArmButton = new JoystickButton(secondary, JoystickConstants.Secondary.forceRaiseArm);
  private final JoystickButton rollerInButton = new JoystickButton(secondary, JoystickConstants.Secondary.rollerIn);

  private final JoystickButton raiseClimberButton = new JoystickButton(secondary, JoystickConstants.Secondary.raiseClimber);
  private final JoystickButton swingArmButton = new JoystickButton(secondary, JoystickConstants.Secondary.swingArm);

  private final JoystickButton ixButton = new JoystickButton(driver, 3);
  private final JoystickButton iyButton = new JoystickButton(driver, 4);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_ArmIntake = new Intake();
  private final Climber s_Climber = new Climber();

  private final TeleopSwerve swerveCommand = new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true);
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(swerveCommand);

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
    // secondary.set
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

    defenseButton.whenPressed(new DefenseModeCommand(s_Swerve, true));
    defenseButton.whenReleased(new DefenseModeCommand(s_Swerve, false));

    /* Secondary Buttons */
    armButton.whenPressed(new SetIntakeArmState(s_ArmIntake, ArmState.LOW));
    armButton.whenPressed(new SetRollerState(s_ArmIntake, RollerState.FORWARD));
    armButton.whenReleased(new SetIntakeArmState(s_ArmIntake, ArmState.HIGH));
    armButton.whenReleased(new SetRollerState(s_ArmIntake, RollerState.OFF));

    lowerArmNoSpinButton.whenPressed(new SetIntakeArmState(s_ArmIntake, ArmState.LOW));
    forceRaiseArmButton.whenPressed(new SetIntakeArmState(s_ArmIntake, ArmState.HIGH));

    rollerInButton.whenPressed(new SetRollerState(s_ArmIntake, RollerState.FORWARD));
    rollerInButton.whenReleased(new SetRollerState(s_ArmIntake, RollerState.OFF));

    shootButton.whenPressed(new SetRollerState(s_ArmIntake, RollerState.BACKWARD));
    shootButton.whenReleased(new SetRollerState(s_ArmIntake, RollerState.OFF));

    raiseClimberButton.whenPressed(new SetClimberElevatorState(s_Climber, frc.robot.subsystems.Climber.ArmState.DOWN, ElevatorState.HIGH));
    raiseClimberButton.whenReleased(new SetClimberElevatorState(s_Climber, frc.robot.subsystems.Climber.ArmState.DOWN, ElevatorState.LOW));

    swingArmButton.whenPressed(new ProgressClimberArmState(s_Climber));

    ixButton.whenPressed(new InstantCommand(() -> {swerveCommand.ix = !swerveCommand.ix;}));
    iyButton.whenPressed(new InstantCommand(() -> {swerveCommand.iy = !swerveCommand.iy;}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TaxiAuto(s_Swerve /*, s_ArmIntake*/);
  }
}