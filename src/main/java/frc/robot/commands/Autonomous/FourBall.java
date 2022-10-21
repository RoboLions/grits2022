// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignShooter;
import frc.robot.commands.AutoDropArm;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveElevatorDown;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FourBall extends SequentialCommandGroup {
 
  public FourBall(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem) {
    super(

      new FollowTrajectory(driveSubsystem, Trajectories.fourBall.toFourthBallPartOne).withTimeout(1.7),

      new ParallelCommandGroup(
        new AutoDropArm(armSubsystem).withTimeout(0.4),
        new AutoIntake(intakeSubsystem).withTimeout(1)
      ),

      new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.3),

      new AutoShoot(shooterSubsystem).withTimeout(2),

      new ParallelCommandGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.fourBall.toFourthBallPartTwo).withTimeout(4.5),
        new AutoIntake(intakeSubsystem).withTimeout(4.5)
      ),

      new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.3),

      new FollowTrajectory(driveSubsystem, Trajectories.fourBall.toFourthBallPartThree).withTimeout(3.5),

      new AutoShoot(shooterSubsystem).withTimeout(2)
    );
  }
}
