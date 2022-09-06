// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private Trajectory m_trajectory;
  
  public FollowTrajectory(DriveSubsystem drivetrain, Trajectory trajectory) {
    driveSubsystem = drivetrain;
    m_trajectory = trajectory;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RamseteController m_disabledRamsete = new RamseteController();
    m_disabledRamsete.setEnabled(false); 
    
    RamseteCommand ramseteCommand = new RamseteCommand(
        m_trajectory, 
        driveSubsystem::getPose,
        m_disabledRamsete, 
        DriveConstants.kDriveKinematics,
        (leftWheelSpeed, rightWheelSpeed) -> {
            driveSubsystem.autoDrive(leftWheelSpeed, rightWheelSpeed);
            System.out.println("left/right speed, " + leftWheelSpeed + ", " + driveSubsystem.getWheelSpeeds().leftMetersPerSecond + 
            ", "+ rightWheelSpeed + ", " + driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
        },
        driveSubsystem
    );

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(m_trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    CommandScheduler.getInstance().schedule(ramseteCommand.andThen(() -> driveSubsystem.autoDrive(0, 0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}