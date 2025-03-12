package frc.robot.command.Auto_Cmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;

public class AutoToReef extends SequentialCommandGroup{
    private final CommandSwerveDrivetrain swerve;
    private final limelight limelight;

    public AutoToReef(CommandSwerveDrivetrain swerve, limelight limelight){
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(swerve, limelight);

        addCommands(null);
    }
    
}
