package frc.robot.subsystems.led;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.led.Led.AnimationType;

public interface LedIO {

    public default void setColor(RGBWColor rgb){}
    
    public default void animationPlayer(AnimationType state){}
    
}
