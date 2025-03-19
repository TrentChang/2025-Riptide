package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class SetZero extends Command{
    private final Arm arm;
    private final Claw claw;
    private final Elevator elevator;
    
    public SetZero(Arm arm, Claw claw, Elevator elevator){
        this.arm = arm;
        this.claw = claw;
        this.elevator = elevator;
        addRequirements(arm, claw, elevator);
    }

    @Override
    public void execute(){
        arm.Arm_Zero();
        claw.Claw_Stop();
        elevator.ELE_Floor();
    }
}
