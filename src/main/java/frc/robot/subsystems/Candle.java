package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    // private final CANdle candle = new CANdle(10);
    // private final RainbowAnimation rainbowAnimation = new RainbowAnimation(100, 1, 167);
    // private final LarsonAnimation climbAnimation = new LarsonAnimation(255, 0, 255);
    

    public Candle() {
    //     candle.configFactoryDefault();

    //     candle.clearAnimation(0);
    //     candle.setLEDs(100, 100, 100);
    // }

    // private void rainbow(){
    //     candle.animate(rainbowAnimation);
    // }

    // public void climb_animation(){
    //     candle.animate(climbAnimation);
    }
    @Override
    public void periodic() {
        //rainbow();
    }
}
