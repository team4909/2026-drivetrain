package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Led extends SubsystemBase {
    
 private final LedIO m_io;
private final RGBWColor kRed= new RGBWColor(255,0,0,0);

 public Led(LedIO io){
    m_io=io;
 }
public Command setRed() {
   return this.run(() -> m_io.setColor(kRed));
}




}
