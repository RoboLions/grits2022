// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.AutoAlignShooter;
import frc.robot.commands.AutoDropArm;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoMoveElevatorDown;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HangarTwoBall extends SequentialCommandGroup {
  /** Creates a new HangarTwoBall. */

  // set 68 inches from the center of the ball
  public HangarTwoBall(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super (
      new ParallelCommandGroup(
        new AutoDropArm(armSubsystem).withTimeout(0.3),
        new FollowTrajectory(driveSubsystem, Trajectories.hangarTwoBall.toSecondBall).withTimeout(1.75), // goes 59 in.
        new AutoIntake(intakeSubsystem).withTimeout(2.5)
      ),
      
      new AutoMoveElevatorDown(shooterSubsystem).withTimeout(0.3),
      
      new AutoShoot(shooterSubsystem).withTimeout(2)
    );
  }
}
