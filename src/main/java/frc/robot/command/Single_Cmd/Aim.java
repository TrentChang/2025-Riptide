// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TargetChooser;
import frc.robot.subsystems.limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Aim extends SequentialCommandGroup {
  private final CommandSwerveDrivetrain swerve;
  private final limelight limelight;
  private final TargetChooser targetChooser;
  private boolean isFinished = false;
  /** Creates a new Aim. */
  public Aim(CommandSwerveDrivetrain swerve, limelight limelight, TargetChooser targetChooser) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.swerve = swerve;
    this.limelight = limelight;
    this.targetChooser = targetChooser;
    addRequirements(swerve, limelight);

    int aprilTagID = (int) LimelightHelpers.getFiducialID("");
    Pose2d Robot_Pose = limelight.getRobotPose().toPose2d();
    // stop when no AprilTag is detected
    if (aprilTagID == -1) {
      isFinished = true;
    }
    // filter AprilTag IDs
    if (!(6 <= aprilTagID && aprilTagID <= 11) && !(17 <= aprilTagID && aprilTagID <= 22)) {
      isFinished = true;
    }
    
    if(!isFinished){
    addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(swerve.driveToPose(targetChooser.identify(aprilTagID, Robot_Pose)));
    addCommands(Commands.runOnce(() -> SmartDashboard.putNumber("X", targetChooser.identify(aprilTagID, Robot_Pose).getX())));
    addCommands(Commands.runOnce(() -> SmartDashboard.putNumber("Y", targetChooser.identify(aprilTagID, Robot_Pose).getY())));
    }
  }
}

