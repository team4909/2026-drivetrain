package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import java.util.Vector;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedIOCandle implements LedIO {

    private static final int kSlot0StartIdx = 0;
    private static final int kSlot0EndIdx = 7;

    private final CANdle m_candle;

    private final RGBWColor kViolet;

    
.3.

    public LedIOCandle() {
        m_candle = new CANdle(1, "rio");
        kViolet = new RGBWColor(0, 0, 0, 0);

        var cfg = new CANdleConfiguration();
        // Disable status LED when being controlled
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(cfg);
    }

    public void setColor(RGBWColor rgb) {

        m_candle.setControl(new SolidColor(0, 7).withColor(rgb));

    }

    // @Override
    // public void animationPlayer(AnimationType state) {
    //     switch (m_anim0State) {
    //         default:
    //         case ColorFlow:
    //             m_candle.setControl(
    //                     new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                             .withColor(kViolet));
    //             break;
    //         case Rainbow:
    //             m_candle.setControl(
    //                     new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
    //             break;
    //         case Twinkle:
    //             m_candle.setControl(
    //                     new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                             .withColor(kViolet));
    //             break;
    //         case TwinkleOff:
    //             m_candle.setControl(
    //                     new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                             .withColor(kViolet));
    //             break;
    //         case Fire:
    //             m_candle.setControl(
    //                     new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
    //             break;
    //     }
    // }

}
