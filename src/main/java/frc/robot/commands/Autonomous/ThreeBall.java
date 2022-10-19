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

// place 55 inches away from first ball
public class ThreeBall extends SequentialCommandGroup {

  public ThreeBall(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem) { 
    super(

      new FollowTrajectory(driveSubsystem, Trajectories.terminalTwoBall.toSecondBall).withTimeout(1.75),

      new ParallelCommandGroup(
        new AutoDropArm(armSubsystem).withTimeout(0.3),
        new AutoIntake(intakeSubsystem).withTimeout(1.4)
      ),

      new ParallelCommandGroup(
        new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.3),
        new AutoAlignShooter(driveSubsystem).withTimeout(0.3)
      ),
      
      new AutoShoot(shooterSubsystem).withTimeout(2),

      // end of 2 ball, begin 3 ball
      new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartOne).withTimeout(3.25),

      new ParallelCommandGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartTwo).withTimeout(2.3),
        new AutoIntake(intakeSubsystem).withTimeout(2.5)
      ),

      new ParallelCommandGroup(
        new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.3)
        //new AutoAlignShooter(driveSubsystem).withTimeout(1.25)  // TODO: FIX ALIGN SHOOTER SOMEHOW
      ),
      
      new AutoShoot(shooterSubsystem).withTimeout(1.6)

      /*new AutoDropArm(armSubsystem),

      new ParallelDeadlineGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.twoBall.toSecondBall),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevatorDown(shooterSubsystem)
      ),

      new AutoShoot(shooterSubsystem).withTimeout(3),

      new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartOne),

      new ParallelDeadlineGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartTwo),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevatorDown(shooterSubsystem) 
      ),

      new AutoShoot(shooterSubsystem).withTimeout(2)*/
    );
  }
}