// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Auto_Cmd;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootCoral extends SequentialCommandGroup {
  /** Creates a new AutoShootCoral. */
  private final Claw claw;
  private final Arm arm;
  private final Elevator elevator;
  // private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  public AutoShootCoral(Claw claw, Arm arm, Elevator elevator) {
    this.claw = claw;
    this.arm = arm;
    this.elevator = elevator;
    // this.commandSwerveDrivetrain= commandSwerveDrivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(claw, arm, elevator);
    
    addCommands(new InstantCommand(() -> elevator.ELE_RL4(), elevator));
    addCommands(new InstantCommand(() -> arm.Arm_RL4(), arm));
    addCommands(new InstantCommand(() -> claw.Claw_Suck(), claw));
    addCommands(new WaitCommand(2));
    addCommands(new InstantCommand(() -> claw.Claw_Shoot(), claw));
    addCommands(new WaitCommand(0.5));
    addCommands(new InstantCommand(() -> claw.Claw_Stop(), claw));
    addCommands(new InstantCommand(() -> elevator.ELE_Floor(), elevator));
    addCommands(new InstantCommand(() -> arm.Arm_StartUp(), arm));
    // addCommands(new InstantCommand(()-> commandSwerveDrivetrain.ResetPigeon()));
  }
}
