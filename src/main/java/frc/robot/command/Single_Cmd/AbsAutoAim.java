// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TargetChooser;
import frc.robot.subsystems.limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AbsAutoAim extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final limelight limelight;
  private final TargetChooser targetChooser;
  private boolean isFinished;
  /** Creates a new autio. */
  public AbsAutoAim(CommandSwerveDrivetrain swerve, limelight limelight, TargetChooser targetChooser) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.targetChooser = targetChooser;
    addRequirements(swerve, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TargetChooser targetChooser;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TargetChooser targetChooser;
    int aprilTagID = (int) LimelightHelpers.getFiducialID("");
    Pose2d Robot_Pose = limelight.getRobotPose().toPose2d();
    // stop when no AprilTag is detected
    if (aprilTagID == -1) {
      isFinished = true;
      return;
    }
    // filter AprilTag IDs
    if (!(6 <= aprilTagID && aprilTagID <= 11) && !(17 <= aprilTagID && aprilTagID <= 22)) {
      isFinished = true;
      return;
    }

    swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(""));
    // Commands.runOnce(() -> targetChooser.identify(aprilTagID, Robot_Pose));
    swerve.driveToPose(targetChooser.identify(aprilTagID, Robot_Pose));
    // Commands.runOnce(() -> System.out.println("here"));
    
    // swerve.driveToPose(new Pose2d(5.766, 3.892, Rotation2d.fromDegrees(180)));
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
