// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
  public RoboLionsPID armPID = new RoboLionsPID();
  private final Pigeon2 imu = RobotMap.intakeIMU;
  public static double MAX_ARM_POWER = 0.16;
  public static double arm_cmd;

  public double arm_pitch_readout = 0;

  public ArmSubsystem() {
    armMotor.setNeutralMode(NeutralMode.Brake);
    armPID.initialize2( 0.2, // Proportional Gain 0.02
                        0.0, // Integral Gain .311
                        0.0, // Derivative Gain
                        10, // Cage Limit
                        1, // Deadband
                        MAX_ARM_POWER, // MaxOutput hard deadband as to what the maximum possible command is
                        true,
                        true
    );
  }

  public void moveArmToPosition(double target_pitch) {
    arm_pitch_readout = getPitch();
    
    double arm_cmd = -armPID.execute((double)target_pitch, (double)arm_pitch_readout);
    // add hard deadband to arm so we don't break it
    if(arm_cmd > MAX_ARM_POWER) {
        arm_cmd = MAX_ARM_POWER;
    } else if(arm_cmd < -MAX_ARM_POWER) {
        arm_cmd = -MAX_ARM_POWER;
    }
    
    System.out.println(target_pitch + ", " + arm_pitch_readout + ", " + arm_cmd);
    //System.out.println("Arm Cmd" + arm_cmd);
    armMotor.set(arm_cmd);
  }

  public void moveArmUp() {
      moveArmToPosition(ArmConstants.UP_POSITION);
  }

  public void moveArmDown() {
      moveArmToPosition(ArmConstants.DOWN_POSITION);
  }

  public void setArmPower(double power) {
      // add hard deadband to arm so we don't break it
      if(power > MAX_ARM_POWER) {
          power = MAX_ARM_POWER;
      } else if(power < -MAX_ARM_POWER) {
          power = -MAX_ARM_POWER;
      }

      /*if((power > 0 && power < 0.1) || (power < 0 && power > -0.1)) {
          power = 0;
      }*/

      armMotor.set(power);
  }

  public void stop() {
      armMotor.set(0);
  }

  public double getPitch() {
    double pitch = imu.getPitch();
    return pitch;
  }
}