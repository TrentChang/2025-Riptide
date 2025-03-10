// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Aim extends Command {
  private final CommandSwerveDrivetrain swerve;
  boolean isFinished = false;

  /** Creates a new Aim. */
  public Aim(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double aprilTagID = LimelightHelpers.getFiducialID("");
    // stop when no AprilTag is detected
    if (aprilTagID == 0) {
      isFinished = true;
      return;
    }
    // filter AprilTag IDs
    if (!(6 <= aprilTagID && aprilTagID <= 11) && !(17 <= aprilTagID && aprilTagID <= 22)) {
      isFinished = true;
      return;
    }

    Pose2d BotPose = LimelightHelpers.getBotPose2d_wpiBlue("");
    Pose2d TargetPose = LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d();
    Pose2d RobotOffset = new Pose2d(0.33, 0.33, swerve.getState().Pose.getRotation());
    // Translation2d offset = new Translation2d()
    // Pose2d DrivePose = new Pose2d(new Translation2d(TargetPose.minus(RobotOffset)), LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d().getRotation());

    // Pose2d TargetPose = new Pose2d(0.481, 0.1585, Rotation2d.fromDegrees(180));

    // swerve.driveToPose(TargetPose);
    // LimelightHelpers.getBotPose3d_TargetSpace("");
    LimelightHelpers.getTargetPose3d_RobotSpace("");
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
