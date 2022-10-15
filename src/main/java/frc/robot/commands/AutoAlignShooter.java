// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignShooter extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  
  public double rotate = 0;
  
  public AutoAlignShooter(final DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offsetX = LimelightSubsystem.getLimelightX();

    double setPoint = 0.0; // final point in degrees
    rotate = (-1) * driveSubsystem.limelightRotationPID.execute(setPoint, offsetX);
  
    System.out.println(rotate); // rotate should be positive
    driveSubsystem.driveWithRotation(0, -rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}