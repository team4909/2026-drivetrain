package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
    private LoggedNetworkNumber m_position = new LoggedNetworkNumber("/Tuning/HoodPosition", 1000);
    
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

    public Command tunableShot (){
        return this.run (() -> m_io.setPosition((int) m_position.get())).withName("tunableShot");
    } 


    // @Override
    // public void periodic() {
    //     Logger.processInputs(getName(), m_inputs);
    // }
}
