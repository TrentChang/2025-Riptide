// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Auto_Cmd;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.TargetChooser;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  private CommandSwerveDrivetrain swerve;
  boolean isFinished = false;

  private final Field2d Pose = new Field2d();
  private Pose2d robotPose, llPose;
  // private PoseEstimate llPose;
  private int aprilTagID;
  private Command driveToPose;

  /** Creates a new AutoAim. */
  public AutoAim(CommandSwerveDrivetrain swerve) { 
    this.swerve = swerve;  
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init");

    aprilTagID = (int)LimelightHelpers.getFiducialID("limelight-two");
    LimelightHelpers.SetRobotOrientation("limelight-two", swerve.getYaw(), 0,0, 0, 0, 0);
    llPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two").pose;
    // llPose = LimelightHelpers.getBotPose2d_wpiBlue("");
    if ((6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22) || (1 <= aprilTagID && aprilTagID <= 2) || (12 <= aprilTagID && aprilTagID <= 13) ){
      if (llPose.getX() == 0 && llPose.getY() == 0) {  // invalid Pose2d data
        robotPose = swerve.getState().Pose;
      } else {
        robotPose = llPose;
        swerve.resetPose(llPose);
      }
      // driveToPose = swerve.driveToPose(TargetChooser.identify(aprilTagID, robotPose));

      driveToPose.initialize();
    } else {
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executed");
    
    driveToPose.execute();
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveToPose.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isFinished || driveToPose.isFinished());
  }
}
