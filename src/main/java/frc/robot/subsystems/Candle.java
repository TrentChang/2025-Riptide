package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    private final CANdle candle = new CANdle(10);
    private final RainbowAnimation rainbowAnimation = new RainbowAnimation(100, 1, 167);
    private final StrobeAnimation IsAutoAim = new StrobeAnimation(0, 0, 0, 0, 0, 0);
    private final LarsonAnimation AutoAim_Finished = new LarsonAnimation(255, 0, 255);
    

    public Candle() {
        candle.configFactoryDefault();

        candle.clearAnimation(0);
        candle.setLEDs(100, 100, 100);
    }

    public void rainbow(){
        candle.animate(rainbowAnimation);
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
    public void periodic() {}
}
