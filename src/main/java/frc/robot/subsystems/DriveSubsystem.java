// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  private static final WPI_TalonFX leftBackMotor = RobotMap.leftBackDriveMotor;
  private static final WPI_TalonFX rightBackMotor = RobotMap.rightBackDriveMotor;
  private static final WPI_TalonFX leftFrontMotor = RobotMap.leftFrontDriveMotor;
  private static final WPI_TalonFX rightFrontMotor = RobotMap.rightFrontDriveMotor;

  private final WPI_Pigeon2 driveIMU = RobotMap.chasisIMU;

  // Odometry class for tracking robot pose
  public final DifferentialDriveOdometry m_odometry;

  public double left_speed_cmd;
  public double right_speed_cmd;

  public double left_speed_feedback;
  public double right_speed_feedback;

  static double lastLinearVelocity = 0;
  static double lastRotateVelocity = 0;

  public DriveSubsystem() {
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID());

    leftBackMotor.setNeutralMode(NeutralMode.Coast);
    rightBackMotor.setNeutralMode(NeutralMode.Coast);

    rightBackMotor.setInverted(false);
    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);

    leftBackMotor.setSensorPhase(true);
    leftFrontMotor.setSensorPhase(true);

    leftFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightFrontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    leftBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightBackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftFrontMotor.configNominalOutputForward(0, 10);
    leftFrontMotor.configNominalOutputReverse(0, 10);
    leftFrontMotor.configPeakOutputForward(1, 10);
    leftFrontMotor.configPeakOutputReverse(-1, 10);
    leftFrontMotor.configNeutralDeadband(0.001, 10);

    rightFrontMotor.configNominalOutputForward(0, 10);
    rightFrontMotor.configNominalOutputReverse(0, 10);
    rightFrontMotor.configPeakOutputForward(1, 10);
    rightFrontMotor.configPeakOutputReverse(-1, 10);
    rightFrontMotor.configNeutralDeadband(0.001, 10);

    leftBackMotor.configNominalOutputForward(0, 10);
    leftBackMotor.configNominalOutputReverse(0, 10);
    leftBackMotor.configPeakOutputForward(1, 10);
    leftBackMotor.configPeakOutputReverse(-1, 10);
    leftBackMotor.configNeutralDeadband(0.001, 10);

    rightBackMotor.configNominalOutputForward(0, 10);
    rightBackMotor.configNominalOutputReverse(0, 10);
    rightBackMotor.configPeakOutputForward(1, 10);
    rightBackMotor.configPeakOutputReverse(-1, 10);
    rightBackMotor.configNeutralDeadband(0.001, 10);

    // Sets how much error is allowed
    leftFrontMotor.configAllowableClosedloopError(0, 0, 10);
    leftBackMotor.configAllowableClosedloopError(0, 0, 10);
    rightFrontMotor.configAllowableClosedloopError(0, 0, 10);
    rightBackMotor.configAllowableClosedloopError(0, 0, 10);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(driveIMU.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(
      driveIMU.getRotation2d(), Math.abs(leftDistanceTravelledInMeters()), Math.abs(rightDistanceTravelledInMeters()));
  }

  public void driveWithRotation(double linearTravelSpeed, double rotateSpeed) {
    // input speed is meters per second, input rotation is bot rotation 
    // speed in meters per second

    double leftSpeed = 0; // meters per second
    double rightSpeed = 0; // meters per second

    // Steps:
    // 1 - decide accel or decel rn
    // 2 - limit commanded velocity based on computed accel limit

    double linearAccel = (linearTravelSpeed - lastLinearVelocity)/0.02;
    double rotateAccel = (rotateSpeed - lastRotateVelocity)/0.02;

    double accelLimit = 1; //meters per second

    // are we accel or decel? part 1
    if (lastLinearVelocity > 0) {
      if (linearAccel < 0) {
        // velocity pos, accel negative, speed dec (slowing down), use decel
        accelLimit = 2; // meters per second
      } else {
        accelLimit = 1;
      }
    } else { // we have negative velocity command
      if (linearAccel > 0) {
        // velocity neg, accel pos, speed dec, decel
        accelLimit = 2;
      } else {
        accelLimit = 1;
      }
    }

    if (lastRotateVelocity > 0) {
      if (rotateAccel < 0) {
        // velocity pos, accel negative, speed dec (slowing down), use decel
        accelLimit = 2; // meters per second
      } else {
        accelLimit = 1;
      }
    } else { // we have negative velocity command
      if (rotateAccel > 0) {
        // velocity neg, accel pos, speed dec, decel
        accelLimit = 2;
      } else {
        accelLimit = 1;
      }
    }

    // part 2: limit velocity based on accelLimit
    if (linearAccel > accelLimit) {
      linearTravelSpeed = lastLinearVelocity + accelLimit*0.03;
    }
    else if (linearAccel < -accelLimit) {
      linearTravelSpeed = lastLinearVelocity - accelLimit*0.03;
    }

    if (rotateAccel > accelLimit) {
      rotateSpeed = lastRotateVelocity + accelLimit*0.03;
    }
    else if (rotateAccel < -accelLimit) {
      rotateSpeed = lastRotateVelocity - accelLimit*0.03;
    }

    lastLinearVelocity = linearTravelSpeed; //meters per second
    lastRotateVelocity = rotateSpeed;

    // this block is helping fix the "penguin hop" of the drivetrain
    // operates on input from joystick
    // pt. 2 A
    if (Math.abs(linearTravelSpeed) < 1.5 && linearTravelSpeed > 0.05) {
      
      if (rotateSpeed > 0.3) {
        leftSpeed = (linearTravelSpeed);
        rightSpeed = (linearTravelSpeed - rotateSpeed);
      } else  if (rotateSpeed < -0.3) {
        leftSpeed = (linearTravelSpeed + rotateSpeed);
        rightSpeed = (linearTravelSpeed);
      } else {
        // original way to do tank drive before trying to get rid of penguin
        leftSpeed = (linearTravelSpeed + rotateSpeed); 
        rightSpeed = (linearTravelSpeed - rotateSpeed);
      }
    } else {
      // original way to do tank drive before trying to get rid of penguin
      leftSpeed = (linearTravelSpeed + rotateSpeed);
      rightSpeed = (linearTravelSpeed - rotateSpeed);
    }

    straightDrive(leftSpeed, rightSpeed);
  }

  public void straightDrive(double leftSpeed, double rightSpeed) {

    left_speed_feedback = getBackLeftEncoderVelocityMetersPerSecond();
    right_speed_feedback = getBackRightEncoderVelocityMetersPerSecond();

    // actual speed command passed 
    left_speed_cmd = leftSpeed; // m/s
    right_speed_cmd = rightSpeed; // m/s

    // ticks/meter * 1 second / 1000 ms * 100 = tick seconds / meter ms
    double driveSpeedPer100MS = (DriveConstants.TICKS_PER_METER * (1.0/1000.0) * 100.0); //tick second/\

    // passing in ticks/100 ms by multiplying (tick seconds / meter ms) * (meter/second) * 100
    leftBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd); 

    // actual speed command passed 
    /*left_speed_cmd = leftSpeed; // m/s
    right_speed_cmd = rightSpeed; // m/s

    // 1 meter/second * 0.1 second/100ms * 1 rev/(2pir meters) * gear ratio/1 rev * 2048 counts/rev
    int driveSpeedPer100MS = (int) ((0.1 * 10.71 * 2048) / (2 * Math.PI * 0.1524));

    leftBackMotor.set(TalonFXControlMode.Velocity, (int) (driveSpeedPer100MS*left_speed_cmd*5));
    rightBackMotor.set(TalonFXControlMode.Velocity, (int) (driveSpeedPer100MS*right_speed_cmd*5));
    */
  }

  public void autoDrive(double leftWheelSpeed, double rightWheelSpeed) {

    // actual speed command passed 
    left_speed_cmd = leftWheelSpeed; // m/s
    right_speed_cmd = rightWheelSpeed; // m/s

    // ticks/meter * 1 second / 1000 ms * 100 = tick seconds / meter ms
    double driveSpeedPer100MS = (DriveConstants.TICKS_PER_METER * (1.0/1000.0) * 100.0); //tick second/\

    // passing in ticks/100 ms by multiplying (tick seconds / meter ms) * (meter/second) * 100
    leftBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*left_speed_cmd); 
    rightBackMotor.set(TalonFXControlMode.Velocity, driveSpeedPer100MS*right_speed_cmd); 
  }

  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public double getHeading() {
    return driveIMU.getYaw();
  }

  // Returns the currently-estimated pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Returns the current wheel speeds of the robot
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Math.abs(getBackLeftEncoderVelocityMetersPerSecond()), 
      Math.abs(getBackRightEncoderVelocityMetersPerSecond()));
  }

  // Resets the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, driveIMU.getRotation2d());
  }

  public double distanceTravelledinTicks() {
    return (getBackLeftEncoderPosition() + getBackRightEncoderPosition()) / 2;
  }

  public double getBackLeftEncoderPosition() {
    return leftBackMotor.getSelectedSensorPosition();
  }

  public double getBackRightEncoderPosition() {
    return rightBackMotor.getSelectedSensorPosition();
  }

  public double getBackLeftEncoderVelocity() {
    return leftBackMotor.getSelectedSensorVelocity();
  }

  public double getBackRightEncoderVelocity() {
    return rightBackMotor.getSelectedSensorVelocity();
  }

  public double getBackLeftEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backLeftVelocityMPS = (leftBackMotor.getSelectedSensorVelocity() * 10);
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    backLeftVelocityMPS = backLeftVelocityMPS * DriveConstants.METERS_PER_TICKS;
    return (backLeftVelocityMPS);
  }

  public double getBackRightEncoderVelocityMetersPerSecond() {
    // getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double backRightVelocityMPS = (rightBackMotor.getSelectedSensorVelocity() * 10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    // Need to have a negative for right velocity since the motors are reversed on
    // the opposite side
    backRightVelocityMPS = backRightVelocityMPS * DriveConstants.METERS_PER_TICKS;
    return (backRightVelocityMPS);
  }

  public double getAverageEncoderVelocityMetersPerSecond() {
    double velocityMPS = (getBackLeftEncoderVelocityMetersPerSecond() + getBackRightEncoderVelocityMetersPerSecond())
        * 0.5;
    return (velocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getBackLeftEncoderPosition() * DriveConstants.METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledInMeters() {
    double right_dist = getBackRightEncoderPosition() * DriveConstants.METERS_PER_TICKS;
    return right_dist;
  }

  public double distanceTravelledinMeters() {
    double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
    return distanceTravelled;
  }

  // Zeroes the heading of the robot
  public void ZeroYaw() {
    driveIMU.setYaw(0, 10);
  }

  public void stop() {
    leftBackMotor.set(0);
    leftFrontMotor.set(0);
  }

  // Returns the turn rate of the robot
  /*public double getTurnRate() {
    return driveIMU.getRate();
  }*/
}