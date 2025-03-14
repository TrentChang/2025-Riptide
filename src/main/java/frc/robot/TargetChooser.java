// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class TargetChooser {
       
    public Pose2d TargetPose = new Pose2d();

    // public static HashMap<Integer, List<Pose2d>> map = new HashMap<>();
    public static HashMap<Integer, List<Pose2d>> map = new HashMap<>();// = new ObjectMapper().readValue("SOMETHING", HashMap.class);
    static {
        // 0.164338 0.45

        // Red Alliance
        map.put(6, Arrays.asList(new Pose2d(13.55, 2.828, Rotation2d.fromDegrees(120.0)), new Pose2d(13.834, 2.992, Rotation2d.fromDegrees(120.0))));
        map.put(7, Arrays.asList(new Pose2d(14.35, 3.886, Rotation2d.fromDegrees(180.0)), new Pose2d(14.35, 4.214, Rotation2d.fromDegrees(180.0))));
        map.put(8, Arrays.asList(new Pose2d(13.834, 5.108, Rotation2d.fromDegrees(-120.0)), new Pose2d(13.55, 5.272, Rotation2d.fromDegrees(-120.0))));
        map.put(9, Arrays.asList(new Pose2d(12.518, 5.272, Rotation2d.fromDegrees(-60.0)), new Pose2d(12.234, 5.108, Rotation2d.fromDegrees(-60.0))));
        map.put(10, Arrays.asList(new Pose2d(11.718, 4.214, Rotation2d.fromDegrees(0.0)), new Pose2d(11.718, 3.886, Rotation2d.fromDegrees(0.0))));
        map.put(11, Arrays.asList(new Pose2d(12.234, 2.992, Rotation2d.fromDegrees(60.0)), new Pose2d(12.518, 2.828, Rotation2d.fromDegrees(60.0))));

        // Blue Alliance
        map.put(17, Arrays.asList(new Pose2d(3.69, 2.992, Rotation2d.fromDegrees(60.0)), new Pose2d(3.974, 2.828, Rotation2d.fromDegrees(60.0))));
        map.put(22, Arrays.asList(new Pose2d(5.006, 2.828, Rotation2d.fromDegrees(120.0)), new Pose2d(5.29, 2.992, Rotation2d.fromDegrees(120.0))));
        map.put(21, Arrays.asList(new Pose2d(5.806, 3.886, Rotation2d.fromDegrees(180.0)), new Pose2d(5.806, 4.214, Rotation2d.fromDegrees(180.0))));
        map.put(20, Arrays.asList(new Pose2d(5.29, 5.108, Rotation2d.fromDegrees(-120.0)), new Pose2d(5.006, 5.272, Rotation2d.fromDegrees(-120.0))));
        map.put(19, Arrays.asList(new Pose2d(3.974, 5.272, Rotation2d.fromDegrees(-60.0)), new Pose2d(3.69, 5.108, Rotation2d.fromDegrees(-60.0))));
        map.put(18, Arrays.asList(new Pose2d(3.174, 4.214, Rotation2d.fromDegrees(0.0)), new Pose2d(3.174, 3.886, Rotation2d.fromDegrees(0.0))));
    }

    private double getDistance(Pose2d p1, Pose2d p2) {
        double deltaX = Math.abs(p1.getX() - p2.getX());
        double deltaY = Math.abs(p1.getY() - p2.getY());
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
     
    private boolean isReef(int aprilTagID) {
        return (6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22);
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

    public Command driveToClosestReef(CommandSwerveDrivetrain swerve) {
        Pose2d robotPose;
        int aprilTagID = (int)LimelightHelpers.getFiducialID("");
        Pose2d llPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        System.out.println("Start");
        if (isReef(aprilTagID)) {
            System.out.println(" IsReef confirmed");
        // useless if-else condition = =

        //     if (llPose.getX() == 0 && llPose.getY() == 0) {  // invalid Pose2d data
        //         System.out.println("  Using Odom");
        //         robotPose = swerve.getState().Pose;
        //     } else {
        //         System.out.println("  Using LL");
        //         robotPose = llPose;
        //         swerve.resetPose(llPose);
        //     }
            robotPose = llPose;
            swerve.resetPose(llPose);
            return swerve.driveToPose(identify(aprilTagID, robotPose));
        }
        return new Command(){};
    }


    // public boolean isFinished(){
    //     double aprilTagID = LimelightHelpers.getFiducialID("");
    //     if (aprilTagID == -1) {
    //             return true;
    //     }
    //     if (!(6 <= aprilTagID && aprilTagID <= 11) && !(17 <= aprilTagID && aprilTagID <= 22)) {
    //             return true;
    //     }
    //     return false;
    // }

//     @Override
//     public void periodic(){
//         double aprilTagID = LimelightHelpers.getFiducialID("");
//         if (aprilTagID != -1 || (6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22)) {
//                 Pose2d Robot_Pose = LimelightHelpers.getBotPose2d_wpiBlue("");
//                 TargetPose = identify((int)aprilTagID, Robot_Pose);
//                 SmartDashboard.putNumber("X", identify((int)aprilTagID, Robot_Pose).getX());
//                 SmartDashboard.putNumber("Y", identify((int)aprilTagID, Robot_Pose).getY());
//                 SmartDashboard.putNumber("targetposX", TargetPose.getX());
//                 SmartDashboard.putNumber("targetposY", TargetPose.getY());
//         }
//         field2d.setRobotPose(TargetPose);
//         SmartDashboard.putData("target",field2d);
//    }
}
