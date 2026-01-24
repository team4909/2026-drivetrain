package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShootingCalculator;
import java.util.function.DoubleSupplier;

public class Hood extends SubsystemBase{
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
    
    public Hood(HoodIO io){
        super("Hood");
        m_io = io;
    }
    
    public Command extendHood (){
        return this.run (() -> m_io.setPosition(2000)).withName("ExtendHood");
    } 

     public Command retractHood (){
        return this.run (() -> m_io.setPosition(1000)).withName("RetractHood");
    } 

    public Command testShotHood (){
        return this.run (() -> m_io.setPosition(1500)).withName("testShotHood");
    } 

    public Command aimFromDistance(DoubleSupplier distanceMeters, ShootingCalculator calculator) {
        return this.run(
                () -> {
                    double distance = distanceMeters.getAsDouble();
                    if (!Double.isFinite(distance)) {
                        return;
                    }
                    m_io.setPosition((int) Math.round(calculator.getHoodPosition(distance)));
                })
            .withName("AimHoodFromTagDistance");
    }

    // @Override
    // public void periodic() {
    //     Logger.processInputs(getName(), m_inputs);
    // }
}