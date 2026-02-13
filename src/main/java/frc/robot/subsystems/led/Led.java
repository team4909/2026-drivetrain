package frc.robot.subsystems.led;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

   private final LedIO m_io;
   private final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
   private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
   private int LEDStart = 0;
   private int LEDEnd = 7;
   private LoggedNetworkNumber animation = new LoggedNetworkNumber("/Tuning/Animation", 0);
   public enum AnimationType {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        Off
    }

   public Led(LedIO io) {
      m_io = io;
   }

   public Command setRed() {
      return this.run(() -> m_io.setColor(kRed));
   }

   public Command setAnimation(AnimationType anim) {
      return this.run(() -> m_io.animationPlayer(anim));
   }



}
