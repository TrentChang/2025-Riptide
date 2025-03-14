// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Auto_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlgae extends Command {
  private CommandSwerveDrivetrain swerve;
  boolean isFinished = false;

  private double aprilTagID;
  private Pose2d robotPose, llPose;
  private Command driveToPose;

  /** Creates a new AutoAlgae. */
  public AutoAlgae(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aprilTagID = (int)LimelightHelpers.getFiducialID("");
    llPose = LimelightHelpers.getBotPose2d_wpiBlue("");
    if ((6 <= aprilTagID && aprilTagID <= 11) || (17 <= aprilTagID && aprilTagID <= 22)) {
      if (llPose.getX() == 0 && llPose.getY() == 0) {  // invalid Pose2d data
        robotPose = swerve.getState().Pose;
      } else {
        robotPose = llPose;
        swerve.resetPose(llPose);
      }
    } else {
      isFinished = true;
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
