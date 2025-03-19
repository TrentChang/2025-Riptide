// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Single_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShoot extends Command {
  private final Arm arm;
  private final Claw claw;
  private final Elevator elevator;

  private double ShootSpeed;
  private double Elevator_Height;
  private double Arm_Angle;
  /** Creates a new CoralShoot. */
  public CoralShoot(Arm arm, Claw claw, Elevator elevator) {
    this.arm = arm;
    this.claw = claw;
    this.elevator = elevator;
    addRequirements(claw, elevator);
  //   // Use addRequirements() here to declare subsystem dependencies.
  }

  // // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator_Height = elevator.getAbsolutePosition();
    Arm_Angle = arm.getArmPos();
    if(Elevator_Height < -50 && Arm_Angle > 0.2){
      ShootSpeed = 0.6;
    }
    else if(-5 < Elevator_Height && Elevator_Height < 50){
      ShootSpeed = 0.3;
    }
    else {
      ShootSpeed = 0.2;
    }
    claw.Claw_Shoot();
  }

  // // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
