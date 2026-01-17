package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public double speed = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setSpeed(double speed);

    public void setBrakeMode(boolean enableBrakeMode);

    // Get Hardware Data to Business Logic
    public void updateInputs(IndexerIOInputsAutoLogged m_inputs);
}