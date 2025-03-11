// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import org.opencv.core.Point;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

/** Add your docs here. */
public class TargetChooser extends SubsystemBase{

    // public static HashMap<Integer, List<Pose2d>> map = new HashMap<>();
    public static HashMap<Integer, List<Pose2d>> map;// = new ObjectMapper().readValue("SOMETHING", HashMap.class);
    static {
        map = new HashMap<>();
        map.put(6, Arrays.asList(new Pose2d(13.529, 2.863, Rotation2d.fromDegrees(120.0)),
                new Pose2d(13.815, 3.027, Rotation2d.fromDegrees(120.0))));
        map.put(7, Arrays.asList(new Pose2d(14.31, 3.885, Rotation2d.fromDegrees(180.0)),
                new Pose2d(14.31, 4.215, Rotation2d.fromDegrees(180.0))));
        map.put(8, Arrays.asList(new Pose2d(13.815, 5.073, Rotation2d.fromDegrees(-120.0)),
                new Pose2d(13.529, 5.237, Rotation2d.fromDegrees(-120.0))));
        map.put(9, Arrays.asList(new Pose2d(12.539, 5.237, Rotation2d.fromDegrees(-60.0)),
                new Pose2d(12.253, 5.073, Rotation2d.fromDegrees(-60.0))));
        map.put(10, Arrays.asList(new Pose2d(11.758, 4.215, Rotation2d.fromDegrees(0.0)),
                new Pose2d(11.758, 3.885, Rotation2d.fromDegrees(0.0))));
        map.put(11, Arrays.asList(new Pose2d(12.253, 3.027, Rotation2d.fromDegrees(60.0)),
                new Pose2d(12.539, 2.863, Rotation2d.fromDegrees(60.0))));

        map.put(17, Arrays.asList(new Pose2d(3.709, 3.027, Rotation2d.fromDegrees(60.0)),
                new Pose2d(3.995, 2.863, Rotation2d.fromDegrees(60.0))));
        map.put(22, Arrays.asList(new Pose2d(4.985, 2.863, Rotation2d.fromDegrees(120.0)),
                new Pose2d(5.271, 3.027, Rotation2d.fromDegrees(120.0))));
        map.put(21, Arrays.asList(new Pose2d(5.766, 3.885, Rotation2d.fromDegrees(180.0)),
                new Pose2d(5.766, 4.215, Rotation2d.fromDegrees(180.0))));
        map.put(20, Arrays.asList(new Pose2d(5.271, 5.073, Rotation2d.fromDegrees(-120.0)),
                new Pose2d(4.985, 5.237, Rotation2d.fromDegrees(-120.0))));
        map.put(19, Arrays.asList(new Pose2d(3.995, 5.237, Rotation2d.fromDegrees(-60.0)),
                new Pose2d(3.709, 5.073, Rotation2d.fromDegrees(-60.0))));
        map.put(18, Arrays.asList(new Pose2d(3.214, 4.215, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.214, 3.885, Rotation2d.fromDegrees(0.0))));
    }

    private double getDistance(Pose2d p1, Pose2d p2) {
        double deltaX = Math.abs(p1.getX() - p2.getX());
        double deltaY = Math.abs(p1.getY() - p2.getY());
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public Pose2d identify(int apriltag, Pose2d currPose) {
        List<Pose2d> candidate = map.get(apriltag);
        double d1 = getDistance(currPose, candidate.get(0));
        double d2 = getDistance(currPose, candidate.get(1));
        if (d1 > d2) {  // d2 is closer than d1
            return candidate.get(1);
        } else {
            return candidate.get(0);
        }
    }
}