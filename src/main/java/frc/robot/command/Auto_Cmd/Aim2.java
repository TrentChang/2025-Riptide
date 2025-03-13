// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.command.Auto_Cmd;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.TargetChooser;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public class Aim2 extends Command {

//   private final CommandSwerveDrivetrain swerve;
//   private boolean isFinished = false;
//   private Pose2d target = null;

//   private Command driveToPose = null;

//   public Aim2(CommandSwerveDrivetrain swerve) {
//     this.swerve = swerve;
//     addRequirements(swerve);    
//   }

//   @Override
//   public void initialize() {
//     int tagId = (int) LimelightHelpers.getFiducialID("");
//     SmartDashboard.putNumber("Tag ID", tagId);

//     // check if the apriltag id is valid, if valid then get the target pose
//     if (tagId != -1 || (6 <= tagId && tagId <= 11) || (17 <= tagId && tagId <= 22)) {
//         Pose2d Robot_Pose = swerve.getPose();

//         // target = TargetChooser.identify((int)tagId, Robot_Pose);
//         driveToPose = swerve.driveToPose(target);
//         driveToPose.initialize();

//         SmartDashboard.putNumber("X", Robot_Pose.getX());
//         SmartDashboard.putNumber("Y", Robot_Pose.getY());
//         SmartDashboard.putNumber("targetposX", target.getX());
//         SmartDashboard.putNumber("targetposY", target.getY());
//     } else {
//         isFinished = true;
//     }
//   }

//   @Override
//   public void execute() {
//     if (target == null || isFinished || driveToPose == null || driveToPose.isFinished()) {
//       return;
//     }
//     driveToPose.execute();

//   }

//   @Override
//   public void end(boolean interrupted) {
//     driveToPose.end(interrupted);
//   }

//   @Override
//   public boolean isFinished() {
//     return isFinished | driveToPose.isFinished();
//   }
// }
