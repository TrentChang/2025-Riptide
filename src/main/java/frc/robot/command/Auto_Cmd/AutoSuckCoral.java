package frc.robot.command.Auto_Cmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.Group_Cmd.CoralStation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
public class AutoSuckCoral extends SequentialCommandGroup{
      /** Creates a new AutoShootCoral. */
  private final Elevator elevator;
  private final Arm arm;
  private final Claw claw;

  // private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  public AutoSuckCoral(Elevator elevator, Claw claw, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
    this.claw = claw;
    addRequirements(arm, claw);
    // addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(new InstantCommand(elevator::ELE_Floor));
    addCommands(new InstantCommand(arm::Arm_Station));
    addCommands(new InstantCommand(claw::Claw_Suck));
    addCommands(new WaitCommand(1.5));
    //addCommands(new InstantCommand(arm::Arm_StartUp));
    addCommands(new InstantCommand(claw::Claw_Stop));
}
}