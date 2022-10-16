// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterHoodConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  private static WPI_VictorSPX frontElevatorMotor = RobotMap.frontElevatorMotor;
  private static WPI_VictorSPX backElevatorMotor = RobotMap.backElevatorMotor;
  private static WPI_TalonFX leftShooterMotor = RobotMap.leftShooterMotor;
  private static WPI_TalonFX rightShooterMotor = RobotMap.rightShooterMotor;
  private static WPI_TalonFX hoodMotor = RobotMap.shooterHoodMotor;

  public static final double LEFT_MOVE_BELT_UP_POWER = 0.5;
  public static final double LEFT_MOVE_BELT_DOWN_POWER = -0.3;
  public static final double RIGHT_MOVE_BELT_UP_POWER = -0.5;
  public static final double RIGHT_MOVE_BELT_DOWN_POWER = 0.3;

  public double shooterError = 0;
  public double hoodError = 0;

  public ShooterSubsystem() {
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    hoodMotor.setNeutralMode(NeutralMode.Coast);

    backElevatorMotor.setNeutralMode(NeutralMode.Coast);
    frontElevatorMotor.setNeutralMode(NeutralMode.Coast);

    leftShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
  
    leftShooterMotor.configNominalOutputForward(0, 10);
    rightShooterMotor.configNominalOutputReverse(0, 10);
    hoodMotor.configNominalOutputReverse(0, 10);

    leftShooterMotor.configNeutralDeadband(0.001, 10);
    rightShooterMotor.configNeutralDeadband(0.001, 10);
    hoodMotor.configNeutralDeadband(0.001, 10);

    rightShooterMotor.configNominalOutputForward(0, 10);
    rightShooterMotor.configNominalOutputReverse(0, 10);
    rightShooterMotor.configPeakOutputForward(1, 10);
    rightShooterMotor.configPeakOutputReverse(-1, 10);

    hoodMotor.configNominalOutputForward(0, 10);
    hoodMotor.configNominalOutputReverse(0, 10);
    hoodMotor.configPeakOutputForward(1, 10);
    hoodMotor.configPeakOutputReverse(-1, 10);

    leftShooterMotor.configNominalOutputForward(0, 10);
    leftShooterMotor.configNominalOutputReverse(0, 10);
    leftShooterMotor.configPeakOutputForward(1, 10);
    leftShooterMotor.configPeakOutputReverse(-1, 10);
    leftShooterMotor.configNeutralDeadband(0.001, 10);

    leftShooterMotor.configAllowableClosedloopError(0, 0, 10);
    rightShooterMotor.configAllowableClosedloopError(0, 0, 10);
    hoodMotor.configAllowableClosedloopError(0, 100, 10);
  }

  public double getLeftEncoderVelocity() {
    return leftShooterMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity() {
    return rightShooterMotor.getSelectedSensorVelocity();
  }
  
  public double getLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double leftVelocityTPS = (leftShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for left velocity since the motors are reversed on
    // the opposite side
    return (leftVelocityTPS * ShooterConstants.METERS_PER_TICKS * -1);
  }

  public double getRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double rightVelocityTPS = (rightShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    return (rightVelocityTPS * ShooterConstants.METERS_PER_TICKS);
  }

  public double getHoodVelocity() {
    return (hoodMotor.getSelectedSensorVelocity());
  }

  
  public double getHoodError() {
    return hoodError;
  }

  public double getHoodVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double hoodVelocityMPS = (hoodMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    hoodVelocityMPS = hoodVelocityMPS * ShooterConstants.METERS_PER_TICKS;
    return (hoodVelocityMPS);
  }


  public double getShooterEncoderVelocity() {
    double shooterEncoderVelocity = ((getLeftEncoderVelocityMetersPerSecond() * -1) + getRightEncoderVelocityMetersPerSecond())/2;
    return shooterEncoderVelocity;
  }

  public double getAverageEncoderVelocityMPS() {
    double velocityMPS = (getRightEncoderVelocityMetersPerSecond() + getLeftEncoderVelocityMetersPerSecond())
                          * 0.5;
    return (velocityMPS);
  }

  public double getAverageEncoderVelocity100MS() {
    double velocity100MS = (getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2;
    return velocity100MS;
  }

  public double getRPMOfLeftShooterFalcon() {
    double RPMOfLFalcon = getLeftEncoderVelocity() / 3.413;
    return RPMOfLFalcon;
  }

  public double getRPMOfRightShooterFalcon() {
    double RPMOfRFalcon = getRightEncoderVelocity() / 3.413;
    return RPMOfRFalcon;
  }

  public double getRPMOfLeftShooterWheels() {
    double RPMOfLShooterWheels = getRPMOfLeftShooterFalcon(); //* 1.33;
    return RPMOfLShooterWheels;
  }

  public double getRPMOfRightShooterWheels() {
    double RPMOfRShooterWheels = getRPMOfRightShooterFalcon(); //* 1.33;
    return RPMOfRShooterWheels;
  }

  public double getVelocityOfFElevator() {
    return frontElevatorMotor.getSelectedSensorVelocity();
  }

  public double getVelocityOfBElevator() {
    return backElevatorMotor.getSelectedSensorVelocity();
  }
  
  public void moveBeltUp() {
    frontElevatorMotor.set(LEFT_MOVE_BELT_UP_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_UP_POWER);
  }
  
  public void stopBelt() {
		frontElevatorMotor.set(0);
    backElevatorMotor.set(0);
	}
  
  public void moveBeltDown() {
    frontElevatorMotor.set(LEFT_MOVE_BELT_DOWN_POWER);
    backElevatorMotor.set(RIGHT_MOVE_BELT_DOWN_POWER);
    leftShooterMotor.set(0.25);
    rightShooterMotor.set(-0.25);
  }

  public void setHoodSpeed(double hoodSpeed) {  
    hoodMotor.set(hoodSpeed);
  }

  public void setHoodMPS(double hoodMPS) {
    hoodError = hoodMPS - getHoodVelocityMetersPerSecond();
    double wheelSpeedPer100MS = (ShooterHoodConstants.TICKS_PER_METER * (1.0/1000.0) * 100.0); //tick second/\
    hoodMotor.set(TalonFXControlMode.Velocity, hoodMPS*wheelSpeedPer100MS);
  }

  // public void setShooterSpeed(double shooterSpeed) {
  //   leftShooterMotor.set(shooterSpeed);
  //   rightShooterMotor.set(shooterSpeed);
  // }

  public double getAverageShooterRPM() {
    double averageShooterRPM = ((-getRPMOfLeftShooterWheels() + getRPMOfRightShooterWheels()) / 2.0);
    return averageShooterRPM;
  }

  public double getShooterError() {
    return shooterError;
  }

  public void setShooterMPS(double shooterMPS) {
    shooterError = shooterMPS - getAverageEncoderVelocityMPS();
    double wheelSpeedPer100MS = (ShooterConstants.TICKS_PER_METER * (1.0/1000.0) * 100.0); //tick second/\

    leftShooterMotor.set(TalonFXControlMode.Velocity, shooterMPS*wheelSpeedPer100MS*-1); // this side negative
    rightShooterMotor.set(TalonFXControlMode.Velocity, shooterMPS*wheelSpeedPer100MS);
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
    hoodMotor.set(0);
    frontElevatorMotor.set(0);
    backElevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
