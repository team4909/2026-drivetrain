package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import java.util.Vector;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedIOCandle implements LedIO {

    private final CANdle m_candle; 

    private final RGBWColor kBlack;

    public LedIOCandle() {
        m_candle = new CANdle(1, "rio");
        kBlack = new RGBWColor(0, 0, 0, 0);
    }

    public void setColor(RGBWColor rgb) {
        
        m_candle.setControl(new SolidColor(0,7).withColor(rgb));

    }

    @Override
    public void flashColor(RGBWColor rgbFlash) {
       
        for(int i = 0; i <= 4; i++) {

            m_candle.setControl(new SolidColor(0,7).withColor(rgbFlash));
            m_candle.setControl(new SolidColor(0,7).withColor(kBlack));

        }

        
    }

}
