package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class SetZero extends Command{
    private final Arm arm;
    private final Coral coral;
    private final Elevator elevator;
    
    public SetZero(Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(arm, coral, elevator);
    }

    @Override
    public void execute(){
        arm.Arm_Zero();
        coral.Coral_Stop();
        elevator.ELE_Floor();
    }
}
