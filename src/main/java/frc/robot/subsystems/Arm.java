package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private final TalonFX Arm_Motor = new TalonFX(ArmConstants.Arm_ID, "mech");

    private final CANcoder Arm_Encoder = new CANcoder(ArmConstants.Arm_Encoder_ID, "mech");
    
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
        Slot0Configs Arm_PIDConfig = new Slot0Configs();
        Arm_PIDConfig.kP = ArmConstants.Arm_P;
        Arm_PIDConfig.kI = ArmConstants.Arm_I;
        Arm_PIDConfig.kD = ArmConstants.Arm_D;
        Arm_PIDConfig.kV = ArmConstants.Arm_F;
        Arm_Motor_Config.apply(Arm_PIDConfig);
    }

    public double getArmPos(){
        return Arm_Encoder.getAbsolutePosition().getValueAsDouble();
        // return Arm_Motor.getPosition().getValueAsDouble();
    }

    // Arm 
    public void Arm_Zero(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Zero));
    }

    public void Arm_StartUp(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_StartUp));
    }

    public void Arm_Station(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_Station));
    }

    public void Arm_RL1(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL1));
    }

    public void Arm_RL2(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL2));
    }

    public void Arm_RL3(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL3));
    }

    public void Arm_RL4(){
        Arm_Motor.setControl(new MotionMagicDutyCycle(ArmConstants.Arm_RL4));
    }

    public void Arm_DOWN(){
        double pos = Arm_Encoder.getPosition().getValueAsDouble();
        Arm_Motor.setControl(new MotionMagicDutyCycle(pos-0.05));
    }

    public void Arm_UP(){
        double pos = Arm_Encoder.getPosition().getValueAsDouble();
        Arm_Motor.setControl(new MotionMagicDutyCycle(pos+0.05));
    }

    public void Arm_Stop(){
        Arm_Motor.set(0);
    }

    public void Arm_Stay(double POS){
        Arm_Motor.setControl(new MotionMagicDutyCycle(POS));
    }

    @Override
    public void periodic(){
        getArmPos();
        SmartDashboard.putNumber("Arm_Pos", getArmPos());
    }
}
