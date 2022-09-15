// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsShooterCalculate;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootShooter extends CommandBase {

  private final static XboxController manipulatorController = RobotContainer.manipulatorController;
  private final static XboxController driverController = RobotContainer.driverController;
  private final ShooterSubsystem shooterSubsystem;
  public static double y = 0; //For calculations
  public static double yVelocity = 0; //For calculations
  public static double xVelocity = 0; //For calculations
  public static double g = -9.8; //Acceleration due to gravity m/s^2
  public static double t = 0; //Time for calculations
  public static double initialVelocity = 0;
  //private static final DigitalInput elevatorSensor1 = RobotMap.elevatorSensor1;
  
  public ShootShooter(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      double x = LimelightSubsystem.getHorizontalDistance();
      y = RoboLionsShooterCalculate.calculate(x);
    yVelocity = Math.sqrt(-2*g*y);
    t = (-yVelocity + Math.sqrt((Math.pow(yVelocity, 2)) - (4*(0.5*g*-1)))) / (2*(0.5*g)); //change the minus to plus if a negative value is returned
    xVelocity = x/t;
    initialVelocity = Math.sqrt((Math.pow(xVelocity, 2)) + Math.pow(yVelocity, 2));
    
    
    if (driverController.getAButtonPressed()) {
      LimelightSubsystem.setVisionProcessor();
    } 
    /*if (driverController.getYButtonPressed()) {
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

    if (manipulatorController.getXButton()) {
      shooterSubsystem.moveBeltUp();
    } else if (manipulatorController.getAButton()) {
      shooterSubsystem.moveBeltDown();
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