// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final XboxController manipulatorController = RobotContainer.manipulatorController;

    public static int wrist_motion_state = 0;

    public MoveArm(ArmSubsystem arm) {
        armSubsystem = arm;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean y = manipulatorController.getYButton();
        // y button // TODO: don't know if we need y button rn
        boolean b = manipulatorController.getBButton();
        // b button = home (bring intake up to home), the emergency hold button

        if (b) { 
            // System.out.println("     ARM TO HOME");
            armSubsystem.setArmToHome(); 
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}