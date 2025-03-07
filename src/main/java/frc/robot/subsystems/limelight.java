package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

 public class limelight extends SubsystemBase{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("");
    NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-two");

    public Field2d LL_Pose;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public int TagID, TagID2;
    public static boolean tag = false;
    public static Transform2d robotOffset;
    public Pose2d avgPose;

    public limelight(){
        LL_Pose = new Field2d();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public boolean getTag(){
        TagID = (int) LimelightHelpers.getFiducialID("");
        TagID2 = (int) LimelightHelpers.getFiducialID("limelight-two");

        if(TagID == -1 && TagID2 == -1){
            return tag = false;
        }
        else if(TagID != -1 || TagID2 != -1){
            return tag = true;
        }
        return false;
    }

    public Pose3d getRobotPose(){
        return LimelightHelpers.getBotPose3d_wpiBlue("");
    }

    public Pose3d getRobotPose_two(){
        return LimelightHelpers.getBotPose3d_wpiBlue("limelight-two");
    }

    public Pose3d robotToTarget(){
        return LimelightHelpers.getBotPose3d_TargetSpace("");
    }

    public double deltaRobotHeadingDeg(){
        return LimelightHelpers.getBotPose2d("").getRotation().getDegrees();
    }    

    // public Pose2d getLLPose(){
    //     PoseEstimate megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    //     PoseEstimate megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");

    //     if(megatag.tagCount < 1 && megatag2.tagCount < 1){
    //         return avgPose = getRobotPose_two().toPose2d();
    //     }
    //     else if(megatag2.tagCount < 1 && megatag.tagCount < 1){
    //         return avgPose = getRobotPose().toPose2d();
    //     }
    //     else if (megatag.tagCount < 1 && megatag2.tagCount < 1){
    //         return avgPose = new Pose2d( (getRobotPose().toPose2d().getX() + getRobotPose_two().toPose2d().getX()) / 2,
    //                                      (getRobotPose().toPose2d().getY() + getRobotPose_two().toPose2d().getY()) / 2,
    //                                       getRobotPose().toPose2d().getRotation().plus(getRobotPose_two().toPose2d().getRotation()).div(2)
    //                                    );
    //     }
    //     else{
    //         return new Pose2d();
    //     }
    // }

    @Override
    public void periodic(){
            getTag();
            getRobotPose();
            getRobotPose_two();
            robotToTarget();
            deltaRobotHeadingDeg();
            // getLLPose();

        LL_Pose.setRobotPose(avgPose);
        // LL_Pose.setRobotPose(getRobotPose_two().toPose2d());
        // SmartDashboard.putData("LL_Pose", LL_Pose);
        SmartDashboard.putNumber("LX", LimelightHelpers.getTX("limelight-two"));
        SmartDashboard.putNumber("LY", LimelightHelpers.getTY("limelight-two"));
        SmartDashboard.putNumber("LR", getRobotPose_two().getRotation().getAngle());
        // SmartDashboard.putNumber("RY", LimelightHelpers.getTargetPose3d_CameraSpace("").getRotation().getY() * 57.3);
        SmartDashboard.putBoolean("getTag", tag);
   }
}
