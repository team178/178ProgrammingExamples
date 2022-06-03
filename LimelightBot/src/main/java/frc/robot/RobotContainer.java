// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.limelightAiming;
import frc.robot.commands.limelightRanging;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import libs.OI.ConsoleController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain;

  private final LimeLight m_limelight;

  //Creates joystick object for the Main (0) and Aux (1) controllers
  private final ConsoleController m_controller_main = new ConsoleController(0);

  // Create SmartDashboard chooser for autonomous routines and drive
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain = new DriveTrain();

    m_limelight = new LimeLight();

    // Configure the button bindings
    configureButtonBindings();

    configureShuffleBoard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drivetrain.setDefaultCommand(
      new ArcadeDrive(m_controller_main::getLeftStickY, m_controller_main::getRightStickX, m_drivetrain, false));
  }

  private void configureShuffleBoard() {
    //Autonomous Chooser Options (How our robot is going to tackle auto)
    m_autoChooser.setDefaultOption("Limelight Getting into Range", new limelightRanging(m_drivetrain, m_limelight, 2.5));
    m_autoChooser.addOption("Limelight Aiming", new limelightAiming(2, m_drivetrain, m_limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}
