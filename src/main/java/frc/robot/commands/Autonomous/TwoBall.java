// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoShootSecondBall;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBall extends SequentialCommandGroup {

  public TwoBall(final DriveSubsystem driveSubsystem) {
    super(

      new ParallelDeadlineGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartOne)
        //new FollowTrajectory(driveSubsystem, Trajectories.twoBall.toSecondBall)
        //new AutoShootSecondBall(shooterSubsystem)
        //new AutoMoveArmDown(armSubsystem),
        //new AutoIntake(intakeSubsystem)
      )

      /*new ParallelDeadlineGroup(
        new FollowTrajectory(driveSubsystem, Trajectories.threeBall.toThirdBallPartTwo)
      )*/

      //new AutoShoot(shooterSubsystem).withTimeout(1)
    );
  }
}
