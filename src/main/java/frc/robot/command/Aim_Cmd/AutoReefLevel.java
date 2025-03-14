// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Aim_Cmd;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import static frc.robot.TargetChooser.reefMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoReefLevel extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final Arm arm;
  private final Elevator elevator;
  private final Supplier<Integer> reefLevelSupplier;
  private final Pose2d targetPose;
  private Pose2d currPose;

  /** Creates a new AutoEle. 
   * 
   * @param aprilTagID The AprilTag ID of the target reef.
   * @param reef 0 = left, 1 = right.
   * @param reefLevelSupplier Elevator's target height. Accepts from 1 to 4.
   */
  public AutoReefLevel(CommandSwerveDrivetrain swerve, Arm arm, Elevator elevator, int aprilTagID, int reef, Supplier<Integer> reefLevelSupplier) {
    this.swerve = swerve;
    this.arm = arm;
    this.elevator = elevator;
    this.reefLevelSupplier = reefLevelSupplier;

    targetPose = reefMap.get(aprilTagID).get(reef);

    addRequirements(arm, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = swerve.getPose();
    // calculate the distance between the robot and the reef
    double deltaX = Math.abs(currPose.getX() - targetPose.getX());
    double deltaY = Math.abs(currPose.getY() - targetPose.getY());
    double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    
    if (distance <= 0.75) {
      int reefLevel = reefLevelSupplier.get();
      switch (reefLevel) {
        case 1:
          arm.Arm_RL1();  
          elevator.ELE_RL1();
          break;
        case 2:
          arm.Arm_RL2();
          elevator.ELE_RL2();
          break;
        case 3:
          arm.Arm_RL3();
          elevator.ELE_RL3();
          break;
        case 4:
          arm.Arm_RL4();
          elevator.ELE_RL4();
          break;
      }
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
