package frc.robot.subsystems;

import java.util.DuplicateFormatFlagsException;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/** 
* Falcon 500 Motor * 1
*/

public class Arm extends SubsystemBase{
    private final TalonFX Arm_Motor = new TalonFX(ArmConstants.Arm_ID, "mech");
    private final CANcoder Arm_Encoder = new CANcoder(ArmConstants.Arm_Encoder_ID, "mech");

    private double ArmPos;
    
    public Arm(){
        var Arm_Motor_Config = Arm_Motor.getConfigurator();

        Arm_Motor.setNeutralMode(NeutralModeValue.Brake);

        Arm_Motor.setInverted(ArmConstants.Arm_Inverted);

        // set feedback sensor as integrated sensor
        Arm_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder).withFeedbackRemoteSensorID(ArmConstants.Arm_Encoder_ID));

        // set maximum acceleration and velocity        
        Arm_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ArmConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ArmConstants.MAX_VELOCITY));
        
        // Arm PIDConfig
        Slot0Configs DOWN_Arm_PIDConfig = new Slot0Configs();
        DOWN_Arm_PIDConfig.kP = ArmConstants.DOWN_Arm_P;
        DOWN_Arm_PIDConfig.kI = ArmConstants.DOWN_Arm_I;
        DOWN_Arm_PIDConfig.kD = ArmConstants.DOWN_Arm_D;
        DOWN_Arm_PIDConfig.kV = ArmConstants.DOWN_Arm_F;
        Arm_Motor_Config.apply(DOWN_Arm_PIDConfig);

        Slot1Configs UP_Arm_PIDConfigs = new Slot1Configs();
        UP_Arm_PIDConfigs.kP = ArmConstants.UP_Arm_P;
        UP_Arm_PIDConfigs.kI = ArmConstants.UP_Arm_I;
        UP_Arm_PIDConfigs.kD = ArmConstants.UP_Arm_D;
        UP_Arm_PIDConfigs.kV = ArmConstants.UP_Arm_F;        
        Arm_Motor_Config.apply(UP_Arm_PIDConfigs);
    }

    public double getArmPos(){
        ArmPos = Arm_Encoder.getAbsolutePosition().getValueAsDouble();
        return ArmPos;
    }

    // Arm Position
    public void Arm_Zero(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Zero).withSlot(0));
    }

    public void Arm_StartUp(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_StartUp).withSlot(0));
    }

    public void Arm_Station(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Station).withSlot(1));
    }

    public void Arm_Barge(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Barge).withSlot(1));
    }

    public void Arm_Algae(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Algae).withSlot(1));
    }

    public void Arm_RL1(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL1).withSlot(0));
    }

    public void Arm_RL2(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL2).withSlot(0));
    }

    public void Arm_RL3(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL3).withSlot(0));
    }

    public void Arm_RL4(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL4).withSlot(0));
    }

    public void Arm_DOWN(){
       Arm_Motor.set(-0.2);
    }

    public void Arm_UP(){
        Arm_Motor.set(0.2);
    }

    /*
    public void Arm_DOWN(){
        double pos = Arm_Encoder.getPosition().getValueAsDouble();
        Arm_Motor.setControl(new MotionMagicDutyCycle(pos-0.05));
    }

    public void Arm_UP(){
        double pos = Arm_Encoder.getPosition().getValueAsDouble();
        Arm_Motor.setControl(new MotionMagicDutyCycle(pos+0.05));
    }
    */

    public void Arm_Stop(){
        Arm_Motor.set(0);
    }

    @Override
    public void periodic(){
        getArmPos();
        SmartDashboard.putNumber("Arm_Pos", getArmPos());
    }
}
