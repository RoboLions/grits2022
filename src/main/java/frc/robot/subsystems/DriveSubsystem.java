// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

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
    private static final double METERS_PER_TICKS = 1 / TICKS_PER_MINUTE;
  
  public DriveSubsystem(){
    
  leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
  rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());
  leftBackMotor.setNeutralMode(NeutralMode.Coast);
  rightBackMotor.setNeutralMode(NeutralMode.Coast);

    leftFrontMotor.configNominalOutputForward(0,10);
    leftFrontMotor.configNominalOutputReverse(0,10);
    leftFrontMotor.configPeakOutputForward(1,10);
    leftFrontMotor.configPeakOutputReverse(-1,10);

    rightFrontMotor.configNominalOutputForward(0,10);
    rightFrontMotor.configNominalOutputReverse(0,10);
    rightFrontMotor.configPeakOutputForward(1,10);
    rightFrontMotor.configPeakOutputReverse(-1,10);

    leftFrontMotor.configNeutralDeadband(0.001,10);
    leftBackMotor.configNeutralDeadband(0.001,10);
    rightFrontMotor.configNeutralDeadband(0.001,10);
    rightBackMotor.configNeutralDeadband(0.001,10);

    leftFrontMotor.setSensorPhase(true);
    rightFrontMotor.setSensorPhase(false);
    leftBackMotor.setSensorPhase(true);
    rightBackMotor.setSensorPhase(false);

    
    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    leftBackMotor.setInverted(false);
    rightBackMotor.setInverted(true);



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
 
  public double getRightBackEncoderPosition(){
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getLeftBackEncoderPosition(){
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double getLeftFrontEncoderPosition(){
    return leftFrontMotor.getSelectedSensorPosition();
  }

  public double getRightFrontEncoderPosition(){
    return rightFrontMotor.getSelectedSensorPosition();
  }


  public double distanceTravelledinTicks() {
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition())/2;
  }

  public double getLeftBackEncoderVelocityMetersPerSecond(){
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity()*10);
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return (leftVelocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getLeftBackEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }

  public void resetEncoders(){
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
  public void setModePercentVoltage(){
    leftFrontMotor.set(ControlMode.PercentOutput,0);
    rightFrontMotor.set(ControlMode.PercentOutput,0);
    leftBackMotor.set(ControlMode.PercentOutput,0);
    rightBackMotor.set(ControlMode.PercentOutput,0);
  }
 public static void drive(double throttle, double rotate){
  leftFrontMotor.set(throttle+rotate);
  rightFrontMotor.set(throttle-rotate);
  leftBackMotor.set(throttle+rotate);
  rightBackMotor.set(throttle-rotate);
 }
  public void stop() {
    drive(0,0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
