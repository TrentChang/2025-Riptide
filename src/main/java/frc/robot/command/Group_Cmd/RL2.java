package frc.robot.command.Group_Cmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class RL2 extends Command {
    // public RL2(Arm arm, Coral coral, Elevator elevator){
    //     addCommands(Commands.runOnce( () -> elevator.ELE_RL2(), elevator));
    //     addCommands(Commands.runOnce( () -> arm.Arm_RL2(), arm));
    // }
    private final Arm arm;
    private final Coral coral;    
    private final Elevator elevator;

    public RL2 (Arm arm, Coral coral, Elevator elevator){
        this.arm = arm;
        this.coral = coral;
        this.elevator = elevator;
        addRequirements(this.arm, this.coral, this.elevator);
    }

    @Override
    public void execute() {
        elevator.ELE_RL2();
        arm.Arm_RL2();
    }

    @Override
    public void end(boolean interrupted) {}
}
