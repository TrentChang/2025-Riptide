package frc.robot.command.Group_Cmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class RL4 extends Command {
    private final Arm arm;
    private final Coral coral;    
    private final Elevator elevator;

    public RL4 (Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    @Override
    public void execute(){
        elevator.ELE_RL4();
        arm.Arm_RL4();
    }
}
