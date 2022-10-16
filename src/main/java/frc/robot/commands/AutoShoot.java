// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends CommandBase {
 
  private final ShooterSubsystem shooterSubsystem;

  private static Timer autoShooterTimer = new Timer();

  public static double counter = 0;
  double shooterSpeed = 0;
  double hoodSpeed = 0;
  
  public AutoShoot(ShooterSubsystem shooter) {
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = LimelightSubsystem.getHorizontalDistance();
    
    shooterSpeed = (0.471 * (Math.pow(x, 2))) + (-0.132 * x) + 19.4;
    hoodSpeed = (1.55 * (Math.pow(x, 2))) + (-4.65 * x) + 16.3;

    shooterSubsystem.setShooterMPS(shooterSpeed);
    shooterSubsystem.setHoodMPS(hoodSpeed);

    if (Math.abs(shooterSubsystem.getShooterError()) < 2 && // original allowed error: 1.5mps
        Math.abs(shooterSubsystem.getHoodError()) < 4) { // original allowed error: 2mps
      counter++;
    } else {
      counter = 0;
      //shooterTimer.start(); // set time to 0 when error > 0.5 for both hood and shooter
    }

    if (counter > 15) {
      shooterSubsystem.moveBeltUp();
      //System.out.println("TIME WHEN SHOT: " + autoShooterTimer.get());
    } else {
      shooterSubsystem.stopBelt();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
