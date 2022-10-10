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

public class ArmSubsystem extends SubsystemBase {
    // TODO: fix pid b/c that's what keeps the arm up (acting as the brake mode), turn off pid when moving from home to ground position
    // we're relying on gravity to make intake fall

    // pid is automatically on when around home position, only turn pid off when manipulator presses the button

  private final WPI_TalonFX armMotor = RobotMap.intakeArmMotor;
  public RoboLionsPID armPID = new RoboLionsPID();
  private final Pigeon2 imu = RobotMap.intakeIMU;
  public static double MAX_ARM_POWER = 0.2; // DON'T PUT ANY FASTER THAN THIS, 0.9 for old intake design
  public static double arm_cmd;

  public double arm_pitch_readout = 0;

  public ArmSubsystem() {
      // this is in coast because brake isn't strong enough to hold it in home position anyway
      armMotor.setNeutralMode(NeutralMode.Coast); 
      // TODO: figure out new PID values
      // keep PID on at the start
      armPID.initialize2( 0.0, // Proportional Gain 0.09
                          0.0, // Integral Gain 0.1
                          0.0, // Derivative Gain
                          10, // Cage Limit
                          1, // Deadband
                          MAX_ARM_POWER, // MaxOutput hard deadband as to what the maximum possible command is
                          true,
                          true
      );
  }
  
  @Override
  public void periodic() {
      // automatically set PID on when in the range of home position
      arm_pitch_readout = getPitch();
      if ((ArmConstants.HOME_POSITION + 20.0) > arm_pitch_readout) { 
        armMotor.setNeutralMode(NeutralMode.Brake);
        System.out.println("ARM PID ON");
        armPID.initialize2( ArmConstants.P, // Proportional Gain 0.09
                            ArmConstants.I, // Integral Gain 0.1
                            ArmConstants.D, // Derivative Gain
                            10, // Cage Limit
                            1, // Deadband
                            MAX_ARM_POWER, // MaxOutput hard deadband as to what the maximum possible command is
                            true,
                            true
        );
      }
  }

  public void moveArmToPosition(double target_pitch) {
      arm_pitch_readout = getPitch();
      double allowableError = 15;
      
    //   double arm_cmd = armPID.execute((double)target_pitch, (double)arm_pitch_readout);
    if (arm_pitch_readout < (target_pitch + allowableError)) {
        arm_cmd = 0;
    } else {
        arm_cmd = target_pitch - arm_pitch_readout;
        // hard deadband to arm so we don't break it
        if(arm_cmd > MAX_ARM_POWER) {
            arm_cmd = MAX_ARM_POWER;
        } 
        else if(arm_cmd < MAX_ARM_POWER) {
            arm_cmd = MAX_ARM_POWER;
        }
    }
      
      System.out.println(target_pitch + ", " + arm_pitch_readout + ", " + arm_cmd);
      System.out.println("Arm Cmd" + arm_cmd);
      armMotor.set(arm_cmd);
  }

  public void setArmToHome() {
    moveArmToPosition(ArmConstants.HOME_POSITION);
    System.out.println(ArmConstants.HOME_POSITION + ", " + arm_pitch_readout + ", " + arm_cmd);
    System.out.println("Arm Cmd: " + arm_cmd);
  }

  public void setArmToGround() { // turn off PID to release intake to the ground
    armPID.initialize2( 0.0, // Proportional Gain
                        0.0, // Integral Gain
                        0.0, // Derivative Gain
                        10, // Cage Limit
                        1, // Deadband
                        MAX_ARM_POWER, // MaxOutput hard deadband as to what the maximum possible command is
                        true,
                        true
    );
  }

  public void setArmPower(double power) {
      // add hard deadband to arm so we don't break it
      if(power > MAX_ARM_POWER) {
          power = MAX_ARM_POWER;
      } else if(power < -MAX_ARM_POWER) {
          power = -MAX_ARM_POWER;
      }

      if((power > 0 && power < 0.25) || (power < 0 && power > -0.25)) {
          power = 0;
      }

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