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
    
    public static Trajectory getToTSecondBall() {
        String trajectoryJSON = "output/TerminalTwoBall.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toTSecondBall = new Trajectory();
        try {
            toTSecondBall = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toTSecondBall;
    }

    public static Trajectory getToHSecondBall() {
        String trajectoryJSON = "output/HangarTwoBall.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toHSecondBall = new Trajectory();
        try {
            toHSecondBall = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toHSecondBall;
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

    public static Trajectory getToFourthBallPartOne() {
        String trajectoryJSON = "output/FourBallPartOne.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toFourBallPartOne = new Trajectory();
        try {
            toFourBallPartOne = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toFourBallPartOne;
    }

    public static Trajectory getToFourthBallPartTwo() {
        String trajectoryJSON = "output/FourBallPartTwo.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toFourBallPartTwo = new Trajectory();
        try {
            toFourBallPartTwo = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toFourBallPartTwo;
    }

    public static Trajectory getToFourthBallPartThree() {
        String trajectoryJSON = "output/FourBallPartThree.wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory toFourBallPartThree = new Trajectory();
        try {
            toFourBallPartThree = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return toFourBallPartThree;
    }

    public static class terminalTwoBall {
        public static final Trajectory toSecondBall = getToTSecondBall();
    }

    public static class hangarTwoBall {
        public static final Trajectory toSecondBall = getToHSecondBall();
    }

    public static class threeBall {
        public static final Trajectory toThirdBallPartOne = getToThirdBallPartOne();
        public static final Trajectory toThirdBallPartTwo = getToThirdBallPartTwo();
    }

    public static class fourBall {
        public static final Trajectory toFourthBallPartOne = getToFourthBallPartOne();
        public static final Trajectory toFourthBallPartTwo = getToFourthBallPartTwo();
        public static final Trajectory toFourthBallPartThree = getToFourthBallPartThree();
    }
}