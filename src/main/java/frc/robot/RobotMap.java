package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.Servo;

public class RobotMap {
    public static final int LEFT_BACK_DRIVE_PORT = 1;
    public static final int RIGHT_BACK_DRIVE_PORT = 2;
    public static final int LEFT_FRONT_DRIVE_PORT = 3;
    public static final int RIGHT_FRONT_DRIVE_PORT = 4;
    public static final int CHASIS_IMU_PORT = 5;

    public static final int LEFT_SHOOTER_MOTOR_PORT = 9;
    public static final int RIGHT_SHOOTER_MOTOR_PORT = 10;
    public static final int BACK_ELEVATOR_PORT = 11;
    public static final int FRONT_ELEVATOR_PORT = 12;
    public static final int HOOD_MOTOR_PORT = 17;

    public static final int RIGHT_CLIMB_PORT = 13; 
    public static final int LEFT_CLIMB_PORT = 14;
    public static final int HIGH_RIGHT_CLIMB_PORT = 15; 
    public static final int HIGH_LEFT_CLIMB_PORT = 16;

    public static WPI_TalonFX leftBackDriveMotor = new WPI_TalonFX(LEFT_BACK_DRIVE_PORT);
    public static WPI_TalonFX rightBackDriveMotor = new WPI_TalonFX(RIGHT_BACK_DRIVE_PORT);
    public static WPI_TalonFX leftFrontDriveMotor = new WPI_TalonFX(LEFT_FRONT_DRIVE_PORT);
    public static WPI_TalonFX rightFrontDriveMotor = new WPI_TalonFX(RIGHT_FRONT_DRIVE_PORT);
    public static WPI_Pigeon2 chasisIMU = new WPI_Pigeon2(CHASIS_IMU_PORT);

    public static WPI_TalonFX leftShooterMotor = new WPI_TalonFX(LEFT_SHOOTER_MOTOR_PORT);
    public static WPI_TalonFX rightShooterMotor = new WPI_TalonFX(RIGHT_SHOOTER_MOTOR_PORT);
    public static WPI_VictorSPX frontElevatorMotor = new WPI_VictorSPX(FRONT_ELEVATOR_PORT);
    public static WPI_VictorSPX backElevatorMotor = new WPI_VictorSPX(BACK_ELEVATOR_PORT);
    public static WPI_TalonFX shooterHoodMotor = new WPI_TalonFX(HOOD_MOTOR_PORT);

    public static WPI_TalonFX rightClimbMotor = new WPI_TalonFX(RIGHT_CLIMB_PORT);
    public static WPI_TalonFX leftClimbMotor = new WPI_TalonFX(LEFT_CLIMB_PORT);
    public static WPI_TalonFX highRightClimbMotor = new WPI_TalonFX(HIGH_RIGHT_CLIMB_PORT);
    public static WPI_TalonFX highLeftClimbMotor = new WPI_TalonFX(HIGH_LEFT_CLIMB_PORT);

    public static Servo servo1 = new Servo(0); //left
    public static Servo servo2 = new Servo(1); //right
}
