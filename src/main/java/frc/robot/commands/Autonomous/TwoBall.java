// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDropArm;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveElevatorUp;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBall extends SequentialCommandGroup {

  public TwoBall(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(

      new AutoDropArm(armSubsystem).withTimeout(0.25),

      new ParallelDeadlineGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.twoBall.toSecondBall),
        new AutoIntake(intakeSubsystem),
        new AutoMoveElevatorUp(shooterSubsystem) // TODO: test to make sure ball does not pop out of shooter
      ),

      new AutoShoot(shooterSubsystem).withTimeout(3)
    );
  }
}
