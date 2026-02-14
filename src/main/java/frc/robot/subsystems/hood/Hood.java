package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShootingCalculator;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Hood extends SubsystemBase{
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
    private LoggedNetworkNumber m_position = new LoggedNetworkNumber("/Tuning/HoodPosition", 1000);

    private double inmin = 32; // min hood angle
    private double inmax = 48; // max hood angle
    private double outmin = 1000.0; // min pulse width
    private double outmax = 2000.0; // max pulse width
    
    public Hood(HoodIO io){
        super("Hood");
        m_io = io;
    }

    public double map(double x) {
        return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
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

    public Command goTo (DoubleSupplier pulseWidth){
        return this.run (() -> m_io.setPosition(((int) pulseWidth.getAsDouble()))).repeatedly().withName("goToHood");
    }

    public Command tunableShot (){
        return this.run (() -> m_io.setPosition((int) m_position.get())).withName("tunableShot");
    } 

    // public double getBallReleaseAngle() {
    //     double pulseWidth = m_io.getPosition();
    //     return (pulseWidth - outmin) * (inmax - inmin) / (outmax - outmin) + inmin;
    // }


    // @Override
    // public void periodic() {
    //     Logger.processInputs(getName(), m_inputs);
    // }
}