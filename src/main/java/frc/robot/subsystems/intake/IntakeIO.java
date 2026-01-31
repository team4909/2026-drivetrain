package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double speed = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setSpeed(double speed);

    public void setBrakeMode(boolean enableBrakeMode);

    /**
     * Optional: set a closed-loop setpoint for an extender motor (rotations).
     * Default no-op so IO implementations that don't have an extender can ignore it.
     * @param rotations rotations of the extender mechanism
     */
    public default void setExtenderSetpoint(double rotations) {}

    // Get Hardware Data to Business Logic
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs);
}
