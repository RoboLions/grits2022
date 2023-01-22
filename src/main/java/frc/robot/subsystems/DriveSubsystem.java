// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  public static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
    public static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
    public static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
    public static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;
   
    private static final double IN_TO_M = 0.254;
    private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; 
    private static final double DIAMETER_INCHES = 5.0; 
    private static final double WHEEL_DIAMETER = DIAMETER_INCHES*IN_TO_M;
    private static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;
    private static final double GEAR_RATIO = 12.75;
    private static final double TICKS_PER_MINUTE = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO)/(WHEEL_CIRCUMFRENCE);
    private static final double METERS_PER_TICK = 1 / TICKS_PER_MINUTE;
  }
  public DriveSubsystem(){
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontMotor.configVelocityMeasurementWindow(16);
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1 , 5, 10);

    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configVelocityMeasurementWindow(16);
    rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1 , 5, 10);

    
    leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackMotor.configVelocityMeasurementWindow(16);
    leftBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1 , 5, 10);

    
    rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackMotor.configVelocityMeasurementWindow(16);
    rightBackMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1 , 5, 10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  resetEncoders();
}