// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Trajectories {
    
    public static Trajectory getToSecondBall() {
        String trajectoryJSON = "output/TwoBall.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toSecondBall = new Trajectory();
        try {
            toSecondBall = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toSecondBall;
    }

    public static Trajectory getToThirdBallPartOne() {
        String trajectoryJSON = "output/ThreeBallPartOne.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toThirdBallPartOne = new Trajectory();
        try {
            toThirdBallPartOne = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toThirdBallPartOne;
    }

    public static Trajectory getToThirdBallPartTwo() {
        String trajectoryJSON = "output/ThreeBallPartTwo.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toThirdBallPartTwo = new Trajectory();
        try {
            toThirdBallPartTwo = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toThirdBallPartTwo;
    }

    public static Trajectory testPath() {
        String trajectoryJSON = "output/Test.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toTest = new Trajectory();
        try {
            toTest = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toTest;
    }

    public static class twoBall {
        // public static final Trajectory toSecondBall = PathPlanner.loadPath("TwoBall", 0.5, 0.5);
        public static final Trajectory toSecondBall = getToSecondBall();
    }

    public static class threeBall {
        //public static final Trajectory toThirdBall = PathPlanner.loadPath("ThreeBall", 0.5, 0.5);
        public static final Trajectory toThirdBallPartOne = getToThirdBallPartOne();
        public static final Trajectory toThirdBallPartTwo = getToThirdBallPartTwo();
    }

    public static class fourBall {
        public static final Trajectory toFourthBall = PathPlanner.loadPath("FourBall", 0.5, 0.5);
    }

    public static class testPath {
        public static final Trajectory toTest = testPath();
    }
}