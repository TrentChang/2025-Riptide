package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

import frc.robot.subsystems.Elevator;

public class Coral extends SubsystemBase{
    // private final DigitalInput Coral_Detect = new DigitalInput(CoralConstants.Coral_Sensor_ID);
    private final TalonFX Coral_Motor = new TalonFX(CoralConstants.Coral_Motor_ID, "mech");

    public boolean getCoral = false;
    public double CoralVelocity;
    public Coral(){
        Coral_Motor.setNeutralMode(NeutralModeValue.Brake);
        Coral_Motor.setInverted(CoralConstants.Coral_Inverted);
    }

    /**
     * Get the sensor value of the sensor.
     * @return Whether the digital sensor is detecting an Algae.
     */
    // public boolean CoralDetected() {
    //     return !Coral_Detect.get();
    // }

    // public void position(){
    //     Coral_Motor.setControl(new MotionMagicDutyCycle(0));
    // }

    // public double getCoralAbsPos(){
    //     return Coral_Encoder.getAbsolutePosition().getValueAsDouble();
    // }

    // Coral Intake
    public void Coral_Suck(){
        Coral_Motor.set(-0.5);
    }

    public void Coral_Shoot(){
        Coral_Motor.set(0.6);
    }

    public void L1CoralShoot(){
        Coral_Motor.set(0.2);
    }

    public void Coral_Stop(){
        Coral_Motor.set(0);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putBoolean("Algae_Detected", CoarlDetected());
        CoralVelocity = Coral_Motor.getRotorVelocity().getValueAsDouble();
        if(Coral_Motor.getSupplyCurrent().getValueAsDouble() > 8){
            getCoral = true;
        }
        else{
            getCoral = false;
        }
        SmartDashboard.putBoolean("Get_Coral", getCoral);
    }
}
