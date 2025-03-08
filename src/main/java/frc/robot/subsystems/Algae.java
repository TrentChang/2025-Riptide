package frc.robot.subsystems;

import java.util.zip.CRC32C;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

// 2 Karken Motor - for 
public class Algae extends SubsystemBase {
    private final TalonFX Algae_Ctrl = new TalonFX(AlgaeConstants.Algae_Ctrl_ID, "rio"); 
    private final TalonFX Algae_Roller = new TalonFX(AlgaeConstants.Algae_Roller_ID, "rio");
        
    public Algae(){
        var Algae_Ctrl_Config = Algae_Ctrl.getConfigurator();

        Algae_Roller.setNeutralMode(NeutralModeValue.Brake);
        Algae_Roller.setInverted(AlgaeConstants.Algae_Roller_Inverted);

        // set feedback sensor as integrated sensor
        Algae_Ctrl_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity        
        Algae_Ctrl_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(AlgaeConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(AlgaeConstants.MAX_VELOCITY));
        
        // Arm PIDConfig
        Slot0Configs Algae_Ctrl_PIDConfig = new Slot0Configs();
        Algae_Ctrl_PIDConfig.kP = AlgaeConstants.Algae_P;
        Algae_Ctrl_PIDConfig.kI = AlgaeConstants.Algae_I;
        Algae_Ctrl_PIDConfig.kD = AlgaeConstants.Algae_D;
        Algae_Ctrl_PIDConfig.kV = AlgaeConstants.Algae_F;
        Algae_Ctrl_Config.apply(Algae_Ctrl_PIDConfig);

        Algae_Ctrl.setPosition(0);
    }

    public double getPosition(){
        return Algae_Ctrl.getPosition().getValueAsDouble();
    }
    
    public void Algae_Zero(){
        Algae_Ctrl.setControl(new MotionMagicDutyCycle(AlgaeConstants.Algae_Zero));
    }

    public void Algae_out(){
       Algae_Ctrl.setControl(new MotionMagicDutyCycle(AlgaeConstants.Algae_Out));
   }

    public void Algae_In(){
        Algae_Ctrl.setControl(new MotionMagicDutyCycle(AlgaeConstants.Algae_In));
    }

    public void Algae_back(){
        Algae_Ctrl.set(0.9);
    }

    public void Algae_Stop(){
        Algae_Ctrl.set(0);
    }

    public void suck(){
        Algae_Roller.set(1);
    }

    public void shoot(){
        Algae_Roller.set(-0.5);
    }

    public void Stop(){
        Algae_Ctrl.set(0);
        Algae_Roller.set(0);
    }

    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Al_Pos", getPosition());
        if(Algae_Ctrl.getPosition().getValueAsDouble() < -1.8 || Algae_Ctrl.getPosition().getValueAsDouble() > 0.1){
            Algae_Ctrl.set(0);
        }
    }
}
