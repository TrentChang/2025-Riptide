// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TargetChooser;
import frc.robot.subsystems.limelight;

public class Aim extends SequentialCommandGroup {
  private final CommandSwerveDrivetrain swerve;
  private final limelight limelight;
  private final TargetChooser targetChooser;
  //private final Field2d m_Field2d;

  public Aim(CommandSwerveDrivetrain swerve, limelight limelight, TargetChooser targetChooser) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.targetChooser = targetChooser;
    addRequirements(swerve, limelight);
    
    addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(swerve.driveToPose(targetChooser.TargetPose));
    // addCommands(swerve.driveToPose(targetChooser.identify((int)LimelightHelpers.getFiducialID(""),
    //                                                       LimelightHelpers.getBotPose2d_wpiBlue("")))); 
  }
}