// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Aim_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignment extends Command {
  private final CommandSwerveDrivetrain swerve;
  private Pose2d BotPos;
  private double Tag_1, Tag_2;
  private Pose2d LLPos_1, LLPos_2;
  private boolean AutoAlign;
  /** Creates a new AutoAling. */
  public AutoAlignment(CommandSwerveDrivetrain swerve) {
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
    Tag_1 = LimelightHelpers.getFiducialID("");
    Tag_2 = LimelightHelpers.getFiducialID("limelight-two");

    LLPos_1 = LimelightHelpers.getBotPose2d_wpiBlue("");
    LLPos_2 = LimelightHelpers.getBotPose2d_wpiBlue("limelight-two");
    if(Tag_1 != -1 && Tag_2 == -1){
      BotPos = LLPos_1;
      AutoAlign = true;
    }
    else if(Tag_1 == -1 && Tag_2 != -1){
      BotPos = LLPos_2;
      AutoAlign = true;
    }
    else if(Tag_1 != -1 && Tag_2 != -1){
      BotPos = new Pose2d((LLPos_1.getX() + LLPos_2.getX()) / 2,
                          (LLPos_1.getY() + LLPos_2.getY()) / 2,
                          Rotation2d.fromDegrees(LLPos_1.getRotation().getDegrees() + LLPos_2.getRotation().getDegrees()));
      AutoAlign = true;
    }
    else {
      BotPos = swerve.getState().Pose;
      AutoAlign = false;
    }

    if(AutoAlign){
      System.out.println("AutoAligned");
    }
    else{
      System.out.println("Did Not Aligned");
    }
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
