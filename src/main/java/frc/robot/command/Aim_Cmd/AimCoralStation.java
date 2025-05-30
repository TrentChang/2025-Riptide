// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Aim_Cmd;

import static frc.robot.TargetChooser.reefMap;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.limelight;

public class AimCoralStation extends SequentialCommandGroup {
  private final Arm arm;
  private final CommandSwerveDrivetrain swerve;
  private final Claw claw;
  private final limelight limelight;
  
  private Pose2d robotPose, llPose, targetPose;
  private final int aprilTagID;

  public AimCoralStation(Arm arm, CommandSwerveDrivetrain swerve, Claw claw, limelight limelight) {
    this.arm = arm;
    this.swerve = swerve;
    this.claw = claw;
    this.limelight = limelight;
    addRequirements(arm, swerve, claw, limelight);
    
    aprilTagID = (int)LimelightHelpers.getFiducialID("limelight-two");
    llPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
    if(llPose != null){
      if ((6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22) || (1 <= aprilTagID && aprilTagID <= 2) || (12 <= aprilTagID && aprilTagID <= 13)) {
      // if ((1 <= aprilTagID && aprilTagID <= 2) || (12 <= aprilTagID && aprilTagID <= 13)) {
        if (llPose.getX() != 0 && llPose.getY() != 0) {  // valid Pose2d data
          robotPose = llPose;

        } else {
          robotPose = swerve.getState().Pose;
        }
      }
    
    double llPose_X = llPose.getX();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isPresent()){
      if(alliance.get() == Alliance.Red){
        if(Math.abs(llPose_X - reefMap.get(1).get(0).getX()) < Math.abs(llPose_X - reefMap.get(2).get(0).getX())){
          targetPose = reefMap.get(1).get(0);
        }
        else{
          targetPose = reefMap.get(2).get(0);        
        }
      }
      else if(alliance.get() == Alliance.Blue){
        if(Math.abs(llPose_X - reefMap.get(12).get(0).getX()) < Math.abs(llPose_X - reefMap.get(13).get(0).getX())){
          targetPose = reefMap.get(12).get(0);        
        }
        else{
          targetPose = reefMap.get(13).get(0);        
        }
      }
    else{
      System.out.println("WARNING: Alliance NOT DETECTED!");
    }
  }
      addCommands(new InstantCommand(() -> claw.Claw_Suck(), claw));
      addCommands(new InstantCommand(() -> arm.Arm_Station(), arm));
      if(targetPose != null){
      // addCommands(swerve.driveToPose(targetPose));
      
      // addCommands(new InstantCommand(() -> swerve.driveToPose(new Pose2d(16.697198, 6.927, Rotation2d.fromDegrees(54)))));
      // addCommands(new InstantCommand(() -> swerve.driveToPose(new Pose2d(0.851154, 6.800, Rotation2d.fromDegrees(126))))); // Blue R 13
      addCommands(new InstantCommand(() -> swerve.driveToPose(new Pose2d(0.851154, 1.200, Rotation2d.fromDegrees(-126))))); // Blue R 12

    }
    }
}
}