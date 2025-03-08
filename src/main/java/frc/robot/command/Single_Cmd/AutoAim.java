// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  private CommandSwerveDrivetrain swerve;
  boolean isFinished = false;

  /** Creates a new AutoAim. */
  public AutoAim(CommandSwerveDrivetrain swerve) {
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
    Pose2d targetReef;
    Pose2d relativeBotPose = LimelightHelpers.getBotPose3d_TargetSpace("").toPose2d();

    Pose2d relativeTagPose = LimelightHelpers.getTargetPose3d_RobotSpace("").toPose2d();

    if (Math.abs(relativeBotPose.getX() - (-0.164)) < Math.abs(relativeBotPose.getX() - 0.164)) {  // Reef L is closer
      targetReef = relativeTagPose.plus(new Transform2d(-0.164, 0, new Rotation2d()));
    } else {                                                                                       // Reef R is closer
      targetReef = relativeTagPose.plus(new Transform2d(0.164, 0, new Rotation2d()));
    }
    // set current pose as origin point
    swerve.resetPose(new Pose2d());
    // apply target pose to the robot
    swerve.driveToPose(targetReef).execute();  // WARNING: can be a bug here (targetReef is null)
    // path following finished, now correct the heading
    Rotation2d rawHeading = new Rotation2d(swerve.getPigeon2().getYaw().getValue());
    swerve.resetRotation(rawHeading);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
