// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class TargetChooser {

    static HashMap<Integer, List<Pose2d>> map;// = new ObjectMapper().readValue("SOMETHING", HashMap.class);
    static {
        map.put(6, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(7, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(8, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(9, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(10, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(11, Arrays.asList(new Pose2d(), new Pose2d()));

        map.put(17, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(18, );
        map.put(19, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(20, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(21, Arrays.asList(new Pose2d(), new Pose2d()));
        map.put(22, Arrays.asList(new Pose2d(), new Pose2d()));

    }

    private List<Pose2d> makeList(double x1, double y1, double r1, double x2, double y2, double r2) [
        return Arrays.asList(new Pose2d(x1, y1, Rotation2d.fromDegree), new Pose2d());
    ]

    public static Pose2d identify(int apriltag, Translation2d currPose) {
        
    }
}
