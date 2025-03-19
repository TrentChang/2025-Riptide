package frc.robot.subsystems;

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
import frc.robot.Constants.ElevatorConstants;

/**
 * Kraken X60 Motor * 2
 */

public class Elevator extends SubsystemBase{

    private final TalonFX Left_Motor = new TalonFX(ElevatorConstants.LeftMotor_ID, "mech");
    private final TalonFX Right_Motor = new TalonFX(ElevatorConstants.RightMotor_ID, "mech");

    private double LastPos = 0; 
    private int rotation = 0;

    public Elevator(){
        var LeftMotorConfig = Left_Motor.getConfigurator();
        var RightMotorConfig = Right_Motor.getConfigurator();

        Left_Motor.setNeutralMode(NeutralModeValue.Brake);
        Right_Motor.setNeutralMode(NeutralModeValue.Brake);

        Left_Motor.setInverted(ElevatorConstants.LeftMotor_Inverted);
        Right_Motor.setInverted(ElevatorConstants.RightMotor_Inverted);



        // set feedback sensor as integrated sensor
        LeftMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
        RightMotorConfig.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        LeftMotorConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        RightMotorConfig.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ElevatorConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ElevatorConstants.MAX_VELOCITY));

        // Sets the mechanism position of the device in mechanism rotations.
        LeftMotorConfig.setPosition(0);
        RightMotorConfig.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ElevatorConstants.P;
        PIDConfig.kI = ElevatorConstants.I;
        PIDConfig.kD = ElevatorConstants.D;
        PIDConfig.kV = ElevatorConstants.F;
        LeftMotorConfig.apply(PIDConfig);
        RightMotorConfig.apply(PIDConfig);
    }

    public double getAbsolutePosition(){
        return Left_Motor.getPosition().getValueAsDouble();
    }

    public void ELE_Floor(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.floor));
    }

    public void ELE_RL1(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L1));
    }

    public void ELE_RL2(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L2));
    }

    public void ELE_RL3(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L3));
    }

    public void ELE_RL4(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.L4));
    }

    public void ELE_Algae(){
        Left_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.Algae));
        Right_Motor.setControl(new MotionMagicDutyCycle(ElevatorConstants.Algae));
    }

    public void ELE_Up(){
        // double pos = Left_Motor.getPosition().getValueAsDouble();
        // Left_Motor.setControl(new MotionMagicDutyCycle(pos-1));
        // Right_Motor.setControl(new MotionMagicDutyCycle(pos-1));
        Left_Motor.set(-0.6);
        Right_Motor.set(-0.6);
    }
    public void ELE_Down(){
    //     double pos = Left_Motor.getPosition().getValueAsDouble();
    //     Left_Motor.setControl(new MotionMagicDutyCycle(pos+1));
    //     Right_Motor.setControl(new MotionMagicDutyCycle(pos+1));
        Left_Motor.set(0.4);
        Right_Motor.set(0.4);
    }

    public void ELE_Stop(){
        Left_Motor.set(0);
        Right_Motor.set(0);
    }

    public void ELE_Stay(double POS){
        Left_Motor.setControl(new MotionMagicDutyCycle(POS));
    }

    @Override
    public void periodic(){
        // if(Left_Motor.getPosition().getValueAsDouble() < -58.5 || Left_Motor.getPosition().getValueAsDouble() > -0.5){
        //     Left_Motor.set(0);
        //     Right_Motor.set(0);
        // }

        getAbsolutePosition();
        SmartDashboard.putNumber("Eleva_pos_L", getAbsolutePosition());

    }
}

