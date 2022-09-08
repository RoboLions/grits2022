// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

public class Trajectories {

    public static class twoBall {
        public static final Trajectory toSecondBall = PathPlanner.loadPath("TwoBall", 0.5, 0.5);
    }

    public static class threeBall {
        public static final Trajectory toThirdBall = PathPlanner.loadPath("ThreeBall", 0.5, 0.5);
    }

    public static class fourBall {
        public static final Trajectory toFourthBall = PathPlanner.loadPath("FourBall", 0.5, 0.5);
    }
}