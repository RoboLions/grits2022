// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int MANIPULALTOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveConstants {
        private static final double IN_TO_M = .0254;
        public static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
        private static final double DIAMETER_INCHES = 6.0; // wheel diameter
        private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
        private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double GEAR_RATIO = 10.71;
        public static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

        public static final double P = 0.15; //0.15;
        public static final double I = 0.001; //0.0005;
        public static final double D = 0; //0.1; 
        public static final double F = 0.05;
    }

    public static final class ShooterConstants {
        public static final double TICKS_PER_METER = (2048.0 * 12.75 * 10.0) / (5.0);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
    }

    public static final class ShooterHoodConstants {
        public static final double P = 0.1; 
        public static final double I = 0.001;
        public static final double D = 5;
        public static final double F = 0.035; // ~12k counts, 75% of max output
    }
}
