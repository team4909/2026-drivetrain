package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShootingCalculator;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Hood extends SubsystemBase{
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
    private LoggedNetworkNumber m_position = new LoggedNetworkNumber("/Tuning/HoodPosition", 0);

    // private double inmin = 32; // min hood angle
    // private double inmax = 48; // max hood angle
    // private double outmin = 1000.0; // min pulse width
    // private double outmax = 2000.0; // max pulse width
    
    public Hood(HoodIO io){
        super("Hood");
        m_io = io;
    }

    // public double map(double x) {
    //     return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
    // }
    
    public Command extendHood (){
        return this.run (() -> m_io.setPosition(2000)).withName("ExtendHood");
    } 

     public Command retractHood (){
        return this.run (() -> m_io.setPosition(0)).withName("RetractHood");
    } 

    // public Command testShotHood (){
    //     return this.run (() -> m_io.setPosition(1500)).withName("testShotHood");
    // } 

    public Command zeroHood(){
        return Commands.race(Commands.waitSeconds(0.75), new InstantCommand(()-> m_io.setSpeed(0.25))).andThen(new InstantCommand(()-> m_io.resetMotorPosition(0)));
    }

    public Command goTo (DoubleSupplier pulseWidth){
        return this.run (() -> m_io.setPosition((pulseWidth.getAsDouble()))).repeatedly().withName("goToHood");
    }

    public Command tunableShot (){
        return this.runOnce (() -> m_io.setPosition(m_position.get())).withName("tunableShot").repeatedly();
    } 

    // public double getBallReleaseAngle() {
    //     double pulseWidth = m_io.getPosition();
    //     return (pulseWidth - outmin) * (inmax - inmin) / (outmax - outmin) + inmin;
    // }


    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);
    }
}