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

        public boolean spindexerConnected = false;
        public double spindexerVelocityRPS = 0.0;
        public double spindexerStatorCurrent = 0.0;
        public double spindexerSupplyCurrent = 0.0;
        public double spindexerVoltage = 0.0;

        public boolean kickerConnected = false;
        public double kickerVelocityRPS = 0.0;
        public double kickerStatorCurrent = 0.0;
        public double kickerSupplyCurrent = 0.0;
        public double kickerVoltage = 0.0;
        
        public double goalVelocity = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setSpeed(double speed);

    public void setBrakeMode(boolean enableBrakeMode);

    // Get Hardware Data to Business Logic
    public void updateInputs(IndexerIOInputsAutoLogged m_inputs);
}