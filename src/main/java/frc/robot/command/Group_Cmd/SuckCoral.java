// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SuckCoral extends Command {
  private final Coral coral;
  private final Arm arm;
  /** Creates a new SuckCoral. */
  public SuckCoral(Coral coral, Arm arm) {
    this.coral = coral;
    this.arm = arm;
    addRequirements(coral, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.Coral_Suck();
    arm.Arm_Station();
    if(coral.CoralVelocity < -50){
      System.out.println("suck");
      coral.Coral_Suck();
      new WaitCommand(0.3);
      if(coral.getCoral){
        coral.Coral_Stop();
      }
      else{
        coral.Coral_Suck();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.Coral_Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coral.getCoral;
  }
}
