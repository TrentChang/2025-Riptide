package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * Kraken X60 Motor * 1
 */

public class Claw extends SubsystemBase{
    private final TalonFX Claw_Motor = new TalonFX(ClawConstants.Claw_Motor_ID, "mech");

    public boolean getClaw = false;
    public double ClawCurrent, ClawVelocity;
    
    public Claw(){
        Claw_Motor.setNeutralMode(NeutralModeValue.Brake);
        Claw_Motor.setInverted(ClawConstants.Claw_Inverted);
    }

    // Claw Intake
    public void Claw_Suck(){
        Claw_Motor.set(-0.5);
    }

    public void Claw_Shoot(){
        Claw_Motor.set(0.6);
    }

    public void L1ClawShoot(){
        Claw_Motor.set(0.2);
    }

    public void Claw_Stop(){
        Claw_Motor.set(0);
    }

    @Override
    public void periodic(){
        // ClawVelocity = Claw_Motor.getRotorVelocity().getValueAsDouble();
        // ClawCurrent = Claw_Motor.getSupplyCurrent().getValueAsDouble();
        // if(ClawCurrent > 40){
        //     getClaw = true;
        // }
        // else{
        //     getClaw = false;
        // }
        // SmartDashboard.putBoolean("Get_Claw", getClaw);
        // SmartDashboard.putNumber("ClawSpeed", ClawVelocity);
    }
}
