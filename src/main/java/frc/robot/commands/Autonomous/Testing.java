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
import frc.robot.commands.StopNWait;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Testing extends SequentialCommandGroup {
  
  // IF align is after FollowTrajectory it doesn't work
  // If align is inside parallel command by itself it works
  // if align is after drop arm it works
  // if align is only 9 degrees offset, it still works
  // if inside follow trajectory take out stop drivetrain at the end of command, did nothing
  // if add stop and wait for 1 sec, did nothing
  // if change autoDrive(0,0) to stop() in follow trajectory, did nothing
  // if increase stopnwait to 3, did nothing
  // if take out stopnwait, did nothing
  // if increase rotate command multiplier from 1 to 1.5, did nothing
  // if increase from 1.5 to 2, helped but we think this is a bad way to go about solving the problem
  // changed back rotate multiplier to 1, if change autoDrive(0,0) inside of end of initial follow trajectory to stop(),did noting
  // if moved follow trajectory stop() to end(), did nothing
  // if bot in same place, take out follow trajectory, will rotate still = 0.2
  // so the rotate cmd out of auto align shooter is correct, but the motors are not following the command
  // if intead of using driveWithRotation put code directly into robot and put follow trajectory back, did nothing

  public Testing(final DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
    super(

      /*new FollowTrajectory(driveSubsystem, Trajectories.hangarTwoBall.toSecondBall).withTimeout(1.75),

      //new AutoShoot(shooterSubsystem).withTimeout(0.5),

      new StopNWait(driveSubsystem, 1),

      new ParallelCommandGroup(
        new AutoAlignShooter(driveSubsystem).withTimeout(1)
      )*/

      new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartTwo).withTimeout(2.3)

    );
  }
}
