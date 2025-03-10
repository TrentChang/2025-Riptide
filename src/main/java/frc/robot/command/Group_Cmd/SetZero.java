package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class SetZero extends Command{
    private final Algae algae;
    private final Arm arm;
    private final Candle candle;
    private final Climber climber;
    private final Coral coral;
    private final Elevator elevator;
    
    public SetZero(Algae algae, Arm arm, Candle candle, Climber climber, Coral coral, Elevator elevator){
        this.algae = algae;
        this.arm = arm;
        this.candle = candle;
        this.climber = climber;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(algae, arm, candle, climber, coral, elevator);
    }

    @Override
    public void execute(){
        // algae.Algae_Zero();
        arm.Arm_Zero();
        // candle.climb_animation();
        // climber.Climb_Zero();
        coral.Coral_Stop();
        elevator.ELE_Floor();
    }
}
