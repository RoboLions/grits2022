// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;

public class Trajectories {

    public static class twoBall {
        public static final Trajectory toFirstBall = PathPlanner.loadPath("Path1", 0.5, 0.5);
    }
}