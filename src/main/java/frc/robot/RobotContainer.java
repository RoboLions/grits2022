// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.MoveClimb;
import frc.robot.commands.ShootShooter;
import frc.robot.commands.Autonomous.TwoBall;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  public static final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static final XboxController manipulatorController = new XboxController(OIConstants.MANIPULATOR_CONTROLLER_PORT);

  public static TwoBall defaultAutoPath = new TwoBall(driveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(
      new JoystickDrive(driveSubsystem)
    );

    shooterSubsystem.setDefaultCommand(
      new ShootShooter(shooterSubsystem)
    );

    climbSubsystem.setDefaultCommand(
      new MoveClimb(climbSubsystem)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return defaultAutoPath;
  }
}
