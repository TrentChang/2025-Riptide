package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class Barge extends Command {
    private final Arm arm;
    private final Claw coral;    
    private final Elevator elevator;

    public Barge (Arm arm, Claw coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    @Override
    public void execute(){
        arm.Arm_Barge();
        elevator.ELE_RL4();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
