package frc.robot.command.Auto_Cmd;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.command.Group_Cmd.SuckCoral;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
public class AutoSuckCoral extends SequentialCommandGroup{
      /** Creates a new AutoShootCoral. */
  private final Coral coral;
  private final SuckCoral suckCoral;
  private final CommandSwerveDrivetrain swerve;
  // private final CommandSwerveDrivetrain commandSwerveDrivetrain;
  public AutoSuckCoral(Coral coral, SuckCoral suckCoral, CommandSwerveDrivetrain swerve) {
    this.coral = coral;
    this.suckCoral = suckCoral;
    this.swerve = swerve;

    //addRequirements(coral);
    addRequirements(swerve);
    addCommands(Commands.runOnce(() -> swerve.resetPose(LimelightHelpers.getBotPose2d_wpiBlue("")), swerve));
    addCommands(new InstantCommand(coral::Coral_Suck));
    addCommands(new WaitCommand(0.5));
    addCommands(suckCoral);
}
}