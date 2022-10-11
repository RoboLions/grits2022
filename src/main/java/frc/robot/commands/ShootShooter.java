// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.RoboLionsShooterCalculate;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final static XboxController driverController = RobotContainer.driverController;
  private final ShooterSubsystem shooterSubsystem;

  private static Timer shooterTimer = new Timer();

  public static double y;
  public static double yVelocity = 0;
  public static double xVelocity = 0;
  public static double t = 0; // Time for calculations
  public static double initialVelocity = 0;
  public static double RPMofFront = 0;
  public static double RPMofHood = 0;
  public static double counter = 0;
  double shooterSpeed = 0;
  double hoodSpeed = 0;

  public ShootShooter(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double x = LimelightSubsystem.getHorizontalDistance();
    y = RoboLionsShooterCalculate.calculate(x);
    yVelocity = Math.sqrt(-2 * ShooterConstants.g * y);
    t = (-yVelocity + Math.sqrt((Math.pow(yVelocity, 2)) - (4*(0.5*ShooterConstants.g*-1)))) / (2*(0.5*ShooterConstants.g)); //change the minus to plus if a negative value is returned
    xVelocity = x/t;
    initialVelocity = Math.sqrt((Math.pow(xVelocity, 2)) + Math.pow(yVelocity, 2)); // meters per second
    RPMofFront = (initialVelocity * 60) / (2 * Math.PI * .0508); //RPM, .0508meters is radius of the wheel
    RPMofHood = (initialVelocity * 60) / (2 * Math.PI * .0254); //RPM, .0254meters is radius of the wheel
    shooterSpeed = (0.471 * (Math.pow(x, 2))) + (-0.132 * x) + 19.4;
    hoodSpeed = (1.55 * (Math.pow(x, 2))) + (-4.65 * x) + 16.3;

    //System.out.println(initialVelocity); // testing purposes
    // System.out.println("shooter error: " + shooterSubsystem.getShooterError()); // testing purposes
    // System.out.println("hood error: " + shooterSubsystem.getHoodError()); // testing purposes

    if (manipulatorController.getXButton()) { // was initially y button 
      // shooterSubsystem.setShooterMPS(31.5);
      shooterSubsystem.setShooterMPS(shooterSpeed);
      // shooterSubsystem.setHoodMPS(68);
      shooterSubsystem.setHoodMPS(hoodSpeed);

      // we just increased the error for now
      if (Math.abs(shooterSubsystem.getShooterError()) < 4.5 && // original allowed error: 1.5mps
          Math.abs(shooterSubsystem.getHoodError()) < 5) { // original allowed error: 2mps
        counter++;
      } else {
        counter = 0;
        shooterTimer.start(); // set time to 0 when error > 0.5 for both hood and shooter
      }
    } else {
      shooterSubsystem.stopShooter();
    }

    /*if (driverController.getAButtonPressed()) {
      LimelightSubsystem.setVisionProcessor();
    } 
    if (driverController.getYButtonPressed()) {
      LimelightSubsystem.setDriverCamera();
    }*/

    // reverse
  /*  if (manipulatorController.getLeftTriggerAxis() > 0.25) {
      shooterSubsystem.setShooterSpeed(-0.35);
    } 
    // shoot upper hub
    else if (manipulatorController.getRightTriggerAxis() > 0.25) {
      shooterSubsystem.steadyShoot(5); //0.94
      shooterSubsystem.setHoodSpeed(0.75); //.65
    } 
    // shoot low goal
    else if (manipulatorController.getRightBumper()) {
      shooterSubsystem.steadyShoot(0.83);
    }
    // shoot without limelight to upper hub from 10 feet
    else if (manipulatorController.getLeftBumper()) {
      shooterSubsystem.steadyShoot(2.65); //2.2
      shooterSubsystem.setHoodSpeed(0.75);
    }
    else {
      shooterSubsystem.stopShooter();
      shooterSubsystem.lastShootVelocity = 0;
    }

    */

    // elevator only goes up when the error is small enough for x number of seconds
    if (manipulatorController.getXButton() && counter > 10) {
      shooterSubsystem.moveBeltUp();
      System.out.println("TIME WHEN SHOT: " + shooterTimer.get());
    } else if (manipulatorController.getAButton()) {
      shooterSubsystem.moveBeltDown();
    } else if (manipulatorController.getLeftBumper()) {
      shooterSubsystem.moveBeltUp();
    } else {
      shooterSubsystem.stopBelt();
    } 
    

    /*if (manipulatorController.getXButton()) {
      if (Math.abs(shooterSubsystem.getLeftEncoderVelocityMetersPerSecond()) < 0.25) {
        if (elevatorSensor1.get() == false) {
          shooterSubsystem.stopBelt();
        } else {
          shooterSubsystem.moveBeltUp();
        }
      } else {
        shooterSubsystem.moveBeltUp();
      }
    } else if (manipulatorController.getAButton()) {
      shooterSubsystem.moveBeltDown();
    } else {
      shooterSubsystem.stopBelt();
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}