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
    double leftVelocityMPS = (leftShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    leftVelocityMPS = leftVelocityMPS * ShooterConstants.METERS_PER_TICKS;
    return (leftVelocityMPS);
  }

  public double getRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double rightVelocityMPS = (rightShooterMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    rightVelocityMPS = rightVelocityMPS * ShooterConstants.METERS_PER_TICKS;
    return (rightVelocityMPS);
  }

  public double getHoodVelocity() {
    return (hoodMotor.getSelectedSensorVelocity());
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

  public double getRPMOfLeftFalcon() {
    double RPMOfLFalcon = getLeftEncoderVelocity() / 3.413;
    return RPMOfLFalcon;
  }

  public double getRPMOfRightFalcon() {
    double RPMOfRFalcon = getRightEncoderVelocity() / 3.413;
    return RPMOfRFalcon;
  }

  public double getRPMOfLeftShooterWheels() {
    double RPMOfLShooterWheels = getRPMOfLeftFalcon() * 1.33;
    return RPMOfLShooterWheels;
  }

  public double getRPMOfRightShooterWheels() {
    double RPMOfRShooterWheels = getRPMOfRightFalcon() * 1.33;
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
  }

  public void setHoodSpeed(double hoodSpeed) {  
    hoodMotor.set(hoodSpeed);
  }

  public void setHoodRPM(double hoodRPM) {

    //don't divide by 100 to keep it 100ms instead of 1 ms
    //int hoodSpeedPer100MS = (int) ((hoodRPM * 0.1 * MOTOR_ENCODER_COUNTS_PER_REV) / (60)); 

    // TODO: check the math
    double hoodSpeedPer100MS = (hoodRPM * 0.1 * ShooterConstants.MOTOR_ENCODER_COUNTS_PER_REV) / (60); 

    hoodMotor.set(TalonFXControlMode.Velocity, hoodSpeedPer100MS);
  }

  public double getHoodRPM() {
    double RPMOfHood = getHoodVelocity() / 3.413;
    return RPMOfHood;
  }

  public void setShooterSpeed(double shooterSpeed) {
    leftShooterMotor.set(shooterSpeed);
    rightShooterMotor.set(shooterSpeed);
  }

  public void setShooterRPM(double shooterRPM) {
    // TODO: check the math
    double shooterSpeedPer100MS = (shooterRPM * 0.1 * ShooterConstants.MOTOR_ENCODER_COUNTS_PER_REV) / (60); 
    leftShooterMotor.set(TalonFXControlMode.Velocity, shooterSpeedPer100MS);
  }

  public void stopShooter() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
