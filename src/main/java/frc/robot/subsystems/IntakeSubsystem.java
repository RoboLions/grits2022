// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  // reversed - sign for DOM
  public static final double IN_POWER = 0.75; // 0.75
  public static final double OUT_POWER = -0.75; // -0.75

  private static final WPI_VictorSPX intakeMotor = RobotMap.intakeRollerMotor;
  private static final WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static final WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  private static final WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static final WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;

  public static final double LEFT_MOVE_BELT_UP_POWER = 0.6;
  public static final double LEFT_MOVE_BELT_DOWN_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_UP_POWER = -0.6;
  public static final double RIGHT_MOVE_BELT_DOWN_POWER = 0.3;

  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void intakeBalls() {
    intakeMotor.set(IN_POWER);
    frontElevatorMotor.set(LEFT_MOVE_BELT_UP_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_UP_POWER);
    //leftShooterMotor.set(0.1);
    //rightShooterMotor.set(-0.1);
  }

  public void outtakeBalls() {
    intakeMotor.set(OUT_POWER);
    frontElevatorMotor.set(LEFT_MOVE_BELT_DOWN_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_DOWN_POWER);
  }

  public void stop() {
    intakeMotor.set(0);
    frontElevatorMotor.set(0);
    backElevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}