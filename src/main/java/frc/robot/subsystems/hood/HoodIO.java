package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO{
    @AutoLog
    public static class HoodIOInputs {
        public double positionLeftActuator = 0.0;

        public double positionRightActuator = 0.0;
    }
    public default void setPosition(int position) {}

    // public void updateInputs(HoodIOInputsAutoLogged m_inputs);




}


