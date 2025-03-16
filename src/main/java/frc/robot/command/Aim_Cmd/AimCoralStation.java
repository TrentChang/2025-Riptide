// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Aim_Cmd;

import static frc.robot.TargetChooser.reefMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.LimelightHelpers;
import frc.robot.TargetChooser;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.limelight;

public class AimCoralStation extends SequentialCommandGroup {
  private final Arm arm;
  private final CommandSwerveDrivetrain swerve;
  private final Coral coral;
  private final limelight limelight;
  private Pose2d robotPose, llPose;
  private final int aprilTagID;
  //private final Field2d m_Field2d;

  public AimCoralStation(Arm arm, CommandSwerveDrivetrain swerve, Coral coral, limelight limelight) {
    this.arm = arm;
    this.swerve = swerve;
    this.coral = coral;
    this.limelight = limelight;
    addRequirements(arm, swerve, coral, limelight);
    
    aprilTagID = (int)LimelightHelpers.getFiducialID("");
    LimelightHelpers.SetRobotOrientation("", swerve.getYaw(), 0, 0, 0, 0, 0);
    llPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose;
    if ((6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22) || (1 <= aprilTagID && aprilTagID <= 2) || (12 <= aprilTagID && aprilTagID <= 13)) {
      if (llPose.getX() == 0 && llPose.getY() == 0) {  // invalid Pose2d data
        robotPose = swerve.getState().Pose;
      } else {
        robotPose = llPose;
      }
      addCommands(new InstantCommand(() -> coral.Coral_Suck(), coral));
      addCommands(new InstantCommand(() -> arm.Arm_Station(), arm));
      addCommands(swerve.driveToPose(reefMap.get(13).get(0)));
    }
  }
}