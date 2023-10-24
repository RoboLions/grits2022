// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class RollIntake extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final static XboxController manipulatorController = RobotContainer.manipulatorController;

  /** Creates a new ManualRollIntake. */
  public RollIntake(IntakeSubsystem intake) {
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (manipulatorController.getAButton()) {
      intakeSubsystem.intakeBalls();
      // System.out.println("A button presed");
    } else {
      intakeSubsystem.stop();      
    }
    // boolean rightTrigger = manipulatorController.getRightTriggerAxis() > 0.25;
    // boolean leftTrigger = manipulatorController.getLeftTriggerAxis() > 0.25;

    /*if (rightTrigger) {
      intakeSubsystem.outtakeBalls();
    } else */
  //   if (leftTrigger) {
  //     intakeSubsystem.intakeBalls();
      
  //   } else {
  //     intakeSubsystem.stop();
  //   }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}