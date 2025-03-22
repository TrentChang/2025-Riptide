// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStation extends Command {
  private final Elevator elevator;
  private final Claw claw;
  private final Arm arm;
  /** Creates a new SuckCoral. 
   *  @param aprilTagID The AprilTag ID of the target reef.
  */
  public CoralStation(Elevator elevator, Claw claw, Arm arm) {
    this.elevator = elevator;
    this.claw = claw;
    this.arm = arm;
    addRequirements(elevator, claw, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.ELE_Floor();
    claw.Claw_Suck();
    arm.Arm_Station();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // coral.Coral_Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return coral.getCoral;
    return false;
  }
}
