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
import frc.robot.subsystems.Coral;
public class AutoSuckCoral extends SequentialCommandGroup{
      /** Creates a new AutoShootCoral. */
  private final Arm arm;
  private final Coral coral;

  // private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  public AutoSuckCoral(Coral coral,Arm arm) {
    this.arm = arm;
    this.coral = coral;
    addRequirements(arm, coral);
    // addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(new InstantCommand(arm::Arm_Station));
    addCommands(new InstantCommand(coral::Coral_Suck));
    addCommands(new WaitCommand(1));
    addCommands(new InstantCommand(arm::Arm_StartUp));
    addCommands(new InstantCommand(coral::Coral_Stop));
}
}