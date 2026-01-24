package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private final HoodIO m_io;
    private final HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();
    
    public Hood(HoodIO io){
        super("Hood");
        m_io = io;
    }
    
    public Command extendHood (double position){
        return this.run (() -> m_io.setPosition(position)).withName("ExtendHood");
    } 


    // @Override
    // public void periodic() {
    //     Logger.processInputs(getName(), m_inputs);
    // }
}
