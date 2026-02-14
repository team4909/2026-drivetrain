package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double speed = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
        public double position = 0.0;

        public String setpoint = "";
    }

    public void setSpeed(double speed);

    public default void setExtenderSetpoint(double rotations) {}

    // Get Hardware Data to Business Logic
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs);

    public void setPosition(double position);
}
