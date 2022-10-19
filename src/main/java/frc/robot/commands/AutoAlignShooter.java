// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignShooter extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  
  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  
  public double rotate = 0;
  public double left_speed_cmd;
  public double right_speed_cmd;
  
  public AutoAlignShooter(final DriveSubsystem drive) {
    driveSubsystem = drive;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("USING LL ALIGN");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offsetX = LimelightSubsystem.getLimelightX();
    //System.out.println("Offset X: " + offsetX);

    double setPoint = 0.0; // final point in degrees
    rotate = (-1) * driveSubsystem.limelightRotationPID.execute(setPoint, offsetX);
   
    //System.out.println("Rotate: " + rotate); // rotate should be positive
    driveSubsystem.driveWithRotation(0, -rotate);

    /*left_speed_cmd = (0 - rotate); 
    right_speed_cmd = (0 + rotate);

    double driveSpeedPer100MS = (DriveConstants.TICKS_PER_METER * (1.0/1000.0) * 100.0);

    leftBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd);*/
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