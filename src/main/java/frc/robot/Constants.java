// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MOTOR_ENCODER_COUNTS_PER_REV = 2048.0;
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int MANIPULATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveConstants {
        private static final double IN_TO_M = .0254;
        // public static final int MOTOR_ENCODER_COUNTS_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
        private static final double DIAMETER_INCHES = 6.0; // wheel diameter
        private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
        private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double GEAR_RATIO = 10.71;
        public static final double TICKS_PER_METER = (Constants.MOTOR_ENCODER_COUNTS_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
        
        // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#running-the-identification-routine
        public static final double kTrackwidthMeters = 0.58;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double P = 0.15; // 0.15
        public static final double I = 0.001; //0.0005;
        public static final double D = 0; //0.1; 
        public static final double F = 0.05;
    }

    public static final class ShooterConstants {
        public static final double TICKS_PER_METER = (Constants.MOTOR_ENCODER_COUNTS_PER_REV * 0.5) / (0.1016 * Math.PI);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

        public static final double P = 0.225; // previous value = 0.1 
        public static final double I = 0.00002; // previous value = 0.001
        public static final double D = 0.0;
        public static final double F = 0.042; // previous value = 0.035, ~12k counts, 75% of max output

        public static final double HEIGHT_OF_GOAL_METERS = 2.44; // change if needed
        public static final double g = -9.8; // Acceleration due to gravity m/s^2
    }

    public static final class ShooterHoodConstants {
        public static final double TICKS_PER_METER = (Constants.MOTOR_ENCODER_COUNTS_PER_REV) / (0.0508 * Math.PI);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
        public static final double P = 0.0; // previous value = 0.1
        public static final double I = 0.0; // previous value = 0.001
        public static final double D = 0.0; // previous value = 5
        public static final double F = 0.0118; // previous value = 0.035, ~12k counts, 75% of max output
    }

    public static final class LimelightConstants {
        //modes for limelight led light
        public static final double FORCE_OFF = 1;
        public static final double FORCE_BLINK = 2;
        public static final double FORCE_ON = 3;
    
        //modes for limelight camera 
        public static final double VISION_PROCESSOR = 0;
        public static final double DRIVER_CAMERA = 1;
        public static final double LLAIMING = 0.035;
        public static final double MOTORGAIN = 0.75;

        public static final double P =  0.02; // 0.007; // 0.02
        public static final double I = 0.015; // 0.0003; // 0.015
        public static final double D = 0.0; // 0.0
    }

    public static final class ArmConstants {
        public static final int TIMEOUT_MS = 10;

        // public static final double UP_POSITION = -88.0; //-55 on old intake design
        // public static final double DOWN_POSITION = 30.0; // 60 on old intake design

        public static final double UP_POSITION = 7.16;
        public static final double DOWN_POSITION = 59.0;

        /* 
        ground:
            Yaw   :178.586 '
            Pitch :-30.0146 '
            Roll  :3.47168 '

        home:
            Yaw   :48.2959 '
            Pitch :-82.2217 '
            Roll  :-53.1738 ' 
        */
    }
}
