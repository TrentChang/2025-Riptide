// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class test extends SequentialCommandGroup {
  private final CommandSwerveDrivetrain swerve;
  /** Creates a new one. */
  public test(CommandSwerveDrivetrain swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.swerve = swerve;
    addRequirements(swerve);
    addCommands(new InstantCommand(() -> System.out.println("Go")));
    addCommands(new InstantCommand(() -> swerve.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))), swerve));
    addCommands(swerve.driveToPose(new Pose2d(1, 0, Rotation2d.fromDegrees(0))));
    addCommands(swerve.driveToPose(new Pose2d(1, 0, Rotation2d.fromDegrees(90))));
    addCommands(swerve.driveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(90))));
    addCommands(swerve.driveToPose(new Pose2d(1, 1, Rotation2d.fromDegrees(180))));
    addCommands(swerve.driveToPose(new Pose2d(0, 1, Rotation2d.fromDegrees(180))));
    addCommands(swerve.driveToPose(new Pose2d(0, 1, Rotation2d.fromDegrees(-90))));
    addCommands(swerve.driveToPose(new Pose2d(1, 0, Rotation2d.fromDegrees(-90))));
    addCommands(swerve.driveToPose(new Pose2d(1, 0, Rotation2d.fromDegrees(0))));
    addCommands(swerve.driveToPose(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0))));
  }
}
