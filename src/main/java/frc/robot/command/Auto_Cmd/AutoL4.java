// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command.Auto_Cmd;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoL4 extends SequentialCommandGroup {
  private final Arm arm;
  private final Elevator elevator;
  /** Creates a new AutoL4. */
  public AutoL4(Arm arm, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arm = arm;
    this.elevator = elevator;
    addCommands(new InstantCommand(() -> elevator.ELE_RL4(), elevator));
    addCommands(new InstantCommand(() -> arm.Arm_RL4(), arm));
  }
}
