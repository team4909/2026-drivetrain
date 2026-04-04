package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        // public double speed = 0.0;
        // public double statorCurrent = 0.0;
        // public double supplyCurrent = 0.0; 
        public String command = "";

        public boolean rollerFloor1Connected = false;
        public double rollerFloor1VelocityRPS = 0.0;
        public double rollerFloor1StatorCurrent = 0.0;
        public double rollerFloor1SupplyCurrent = 0.0;
        public double rollerFloor1Voltage = 0.0;

        public boolean rollerFloor2Connected = false;
        public double rollerFloor2VelocityRPS = 0.0;
        public double rollerFloor2StatorCurrent = 0.0;
        public double rollerFloor2SupplyCurrent = 0.0;
        public double rollerFloor2Voltage = 0.0;
        
        public boolean rollerFloor3Connected = false;
        public double rollerFloor3VelocityRPS = 0.0;
        public double rollerFloor3StatorCurrent = 0.0;
        public double rollerFloor3SupplyCureent = 0.0;
        public double rollerFloor3Voltage = 0.0;

    
        public double goalVelocity = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setSpeed(double speed);

    public void setBrakeMode(boolean enableBrakeMode);

    public void setVelocity(double rps);

    // Get Hardware Data to Business Logic
    public void updateInputs(IndexerIOInputsAutoLogged m_inputs);
}