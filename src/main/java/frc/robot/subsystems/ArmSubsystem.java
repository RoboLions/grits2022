// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;

/*
1. Bring arm down at the start of the match, so apply some power for only a tiny bit of time
2. If some button pressed, bring the arm back up while the button is held
both intake and elevator must run at the same time
*/

public class ArmSubsystem extends SubsystemBase {
   
  private final WPI_TalonFX armMotor = RobotMap.intakeArmMotor;

  public static double MAX_ARM_POWER = 0.2; // DON'T PUT ANY FASTER THAN THIS

  public ArmSubsystem() {
    // this is in coast because brake isn't strong enough to hold it in home position anyway
    armMotor.setNeutralMode(NeutralMode.Coast); 
  }

  @Override
  public void periodic() {
  }

  public void dropArm() {
    armMotor.set(0.1);
  }

  public void moveArmUp() {
    armMotor.set(0.15);
  }

  public void stop() {
    armMotor.set(0);
  }
}