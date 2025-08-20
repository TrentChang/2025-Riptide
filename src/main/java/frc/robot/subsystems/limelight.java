package frc.robot.subsystems;

import com.fasterxml.jackson.databind.introspect.AnnotationCollector.TwoAnnotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

 public class limelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
    NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-two");

    public Field2d LL_Pose;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public int TagID, TagID2;
    public static boolean tag = false;
    public static Transform2d robotOffset;
    public Pose2d avgPose;

    public limelight(){
        LL_Pose = new Field2d();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public boolean getTag(){
        TagID = (int) LimelightHelpers.getFiducialID("limelight-two");
        TagID2 = (int) LimelightHelpers.getFiducialID("limelight-two");

        if(TagID == -1 && TagID2 == -1){
            return tag = false;
        }
        else if(TagID != -1 || TagID2 != -1){
            return tag = true;
        }
        return false;
        // if(TagID != -1){
        //     return tag = true;
        // }
        // else{
        //     return tag = false;
        // }
    }

    public Pose2d getRobotPose(){
        TagID = (int) LimelightHelpers.getFiducialID("limelight-two");
        // TagID2 = (int) LimelightHelpers.getFiducialID("limelight-two");
        Pose2d pose1 = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
        // Pose2d pose2 = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
        if(TagID != -1){
            return LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
        }
        // else if(TagID2 != -1){
        //     return LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
        // }
        // else if(TagID != -1 && TagID2 != -1){
        //     return new Pose2d((pose1.getX() + pose2.getX()) / 2,
        //                       (pose1.getY() + pose2.getY()) / 2,
        //                        pose1.getRotation().plus(pose2.getRotation()).div(2)
        //                      );
        // }
        else{
            return new Pose2d();
        }
    }

    // public Pose3d getRobotPose_two(){
    //     return LimelightHelpers.getBotPose3d_wpiBlue("limelight-two");
    // }

    // public Pose3d robotToTarget(){
    //     return LimelightHelpers.getBotPose3d_TargetSpace("");
    // }

    // public double deltaRobotHeadingDeg(){
    //     return LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
    // }  

    @Override
    public void periodic(){
        getTag();
        getRobotPose();
        // LL_Pose.setRobotPose(avgPose);
        LL_Pose.setRobotPose(getRobotPose());
        SmartDashboard.putData("LL_Pose", LL_Pose);
        SmartDashboard.putBoolean("getTag", tag);
        // SmartDashboard.putNumber("LX", LimelightHelpers.getTX("limelight-two"));
        // SmartDashboard.putNumber("LY", LimelightHelpers.getTY("limelight-two"));
        // SmartDashboard.putNumber("LR", getRobotPose_two().getRotation().getAngle());
        // SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY() * 57.3);
        SmartDashboard.putNumber("Offset", LimelightHelpers.getTargetPose3d_RobotSpace("limelight-two").getY());
   }
}
