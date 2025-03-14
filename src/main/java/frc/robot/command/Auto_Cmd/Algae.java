// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Auto_Cmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Algae extends SequentialCommandGroup {
  private final CommandSwerveDrivetrain swerve;
  /** Creates a new Algae. */
  public Algae(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(swerve.driveToPose(new Pose2d(5.766, 4.05, Rotation2d.fromDegrees(180.0))));
  }
}
