package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Kraken X60 Motor * 1
 */

public class Climber extends SubsystemBase{
    private final TalonFX Climber_Motor = new TalonFX(ClimberConstants.Climb_Motor, "mech");

    public Climber(){
        var Climber_Motor_Config = Climber_Motor.getConfigurator();

        Climber_Motor.setNeutralMode(NeutralModeValue.Brake);

        Climber_Motor.setInverted(ClimberConstants.LeftMotor_Inverted);

        // set feedback sensor as integrated sensor
        Climber_Motor_Config.apply(new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

        // set maximum acceleration and velocity
        Climber_Motor_Config.apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.MAX_ACCEL)
                .withMotionMagicCruiseVelocity(ClimberConstants.MAX_VELOCITY));
        
        Climber_Motor_Config.setPosition(0);

        // PIDConfig
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kP = ClimberConstants.P;
        PIDConfig.kI = ClimberConstants.I;
        PIDConfig.kD = ClimberConstants.D;
        PIDConfig.kV = ClimberConstants.F;
        Climber_Motor_Config.apply(PIDConfig);

        // Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_StartUp));

        if(DriverStation.isEnabled()){
            Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_StartUp));
        }
    }

    public double getAbsolutePosition(){
        return Climber_Motor.getPosition().getValueAsDouble();
    }

    public void Climb_Zero(){
        Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Zero));
    }

    public void Climb(){
        Climber_Motor.setControl(new MotionMagicDutyCycle(ClimberConstants.Climb_Angle));
    }

    public void Up(){
        Climber_Motor.set(0.5);
    }

    public void Down(){
        Climber_Motor.set(-0.5);
    }

    public void Stop(){
        Climber_Motor.set(0);
    }
    @Override 
    public void periodic(){
        getAbsolutePosition();
        SmartDashboard.putNumber("Climb_Pos", getAbsolutePosition());
    }
}
