package frc.robot.subsystems;

import java.util.zip.CRC32C;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/*
 * Falcon 500 Motor * 2
 */

public class Intake extends SubsystemBase {
    private final TalonFX Intake_Ctrl = new TalonFX(IntakeConstants.Intake_Ctrl_ID, "rio"); 
    private final TalonFX Intake_Roller = new TalonFX(IntakeConstants.Intake_Roller_ID, "rio");
        
    public Intake(){
        var Intake_Ctrl_Config = Intake_Ctrl.getConfigurator();

        Intake_Roller.setNeutralMode(NeutralModeValue.Brake);
        Intake_Roller.setInverted(IntakeConstants.Intake_Roller_Inverted);

        // set feedback sensor as integrated sensor
        Intake_Ctrl_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity        
        Intake_Ctrl_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(IntakeConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(IntakeConstants.MAX_VELOCITY));
        
        // Arm PIDConfig
        Slot0Configs Intake_Out_PIDConfig = new Slot0Configs();
        Intake_Out_PIDConfig.kP = IntakeConstants.Intake_Out_P;
        Intake_Out_PIDConfig.kI = IntakeConstants.Intake_Out_I;
        Intake_Out_PIDConfig.kD = IntakeConstants.Intake_Out_D;
        Intake_Out_PIDConfig.kV = IntakeConstants.Intake_Out_F;
        Intake_Ctrl_Config.apply(Intake_Out_PIDConfig);

        Slot1Configs Intake_Back_PIDConfig = new Slot1Configs();
        Intake_Back_PIDConfig.kP = IntakeConstants.Intake_Back_P;
        Intake_Back_PIDConfig.kI = IntakeConstants.Intake_Back_I;
        Intake_Back_PIDConfig.kD = IntakeConstants.Intake_Back_D;
        Intake_Back_PIDConfig.kV = IntakeConstants.Intake_Back_F;
        Intake_Ctrl_Config.apply(Intake_Back_PIDConfig);

        Intake_Ctrl.setPosition(0);
    }

    public double getPosition(){
        return Intake_Ctrl.getPosition().getValueAsDouble();
    }
    
    // Intake Position
    public void Intake_Zero(){
        Intake_Ctrl.setControl(new MotionMagicDutyCycle(IntakeConstants.Intake_Zero));
    }

    public void Intake_out(){
       Intake_Ctrl.setControl(new MotionMagicDutyCycle(IntakeConstants.Intake_Out));
   }

    public void Intake_Back(){
        Intake_Ctrl.setControl(new MotionMagicDutyCycle(IntakeConstants.Intake_In).withSlot(1));
    }

    public void Intake_back(){
        Intake_Ctrl.set(0.9);
    }

    public void Intake_Stop(){
        Intake_Ctrl.set(0);
    }

    public void suck(){
        Intake_Roller.set(1);
    }

    public void shoot(){
        Intake_Roller.set(-0.5);
    }

    public void Stop(){
        Intake_Ctrl.set(0);
        Intake_Roller.set(0);
    }

    @Override 
    public void periodic(){
        SmartDashboard.putNumber("Al_Pos", getPosition());
        // if(Intake_Ctrl.getPosition().getValueAsDouble() < -1.8 || Intake_Ctrl.getPosition().getValueAsDouble() > 0.1){
        //     Intake_Ctrl.set(0);
        // }
    }
}
