package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO{
    @AutoLog
    public static class HoodIOInputs {
        // public double positionLeftActuator = 0.0;

        // public double positionRightActuator = 0.0;

        public boolean hoodConnected = false;
        public double hoodVelocityRPS = 0.0;
        public double hoodStatorCurrent = 0.0;
        public double hoodSupplyCurrent = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodRotations = 0.0;

        public double goalPosition = 0.0;
    }
    // public default void setPosition(int position) {}

     public default void setPosition(double position) {}
     public default void setSpeed(double speed) {}
     public default void resetMotorPosition(double rotations) {}

    public default void updateInputs(HoodIOInputsAutoLogged m_inputs) {}

    // public void updateInputs(HoodIOInputsAutoLogged m_inputs);




}


