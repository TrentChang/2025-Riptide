package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    private final CANdle candle = new CANdle(10);
    private final RainbowAnimation Normal = new RainbowAnimation(100, 1, 167);
    private final StrobeAnimation TagGet = new StrobeAnimation(0, 255, 0);
    private final StrobeAnimation IsAutoAim = new StrobeAnimation(255, 0, 255);
    private final LarsonAnimation AutoAim_Finished = new LarsonAnimation(0, 0, 155, 0, 0.8, 1, BounceMode.Center, 5);
    
    private final limelight limelight = new limelight();
    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(100, 100, 100);
    }

    public void Normal(){
        candle.animate(Normal);
    }

    // Tag Get
    public void TagGet(){
        candle.animate(TagGet);
    }
    // Is Auto Aiming 
    public void IsAutoAim(){
        candle.animate(IsAutoAim);
    }

    // Auto Aim Finished
    public void AutoAim_Finished(){
        candle.animate(AutoAim_Finished);
    }
    @Override
    public void periodic() {
        if(limelight.getTag()){
            candle.animate(TagGet);
        }
        else{
            candle.animate(Normal);
        }
    }
}
