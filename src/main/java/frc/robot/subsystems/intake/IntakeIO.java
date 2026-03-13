package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double statorCurrentIntakeRollerRight = 0.0;
        public double statorCurrentIntakeRollerLeft = 0.0;
        public double statorCurrentIntakeExtender = 0.0;

        public double IntakeExtenderPosition = 0.0;

        public double intakeRollerRightVelocity = 0.0;
        public double intakeRollerLeftVelocity = 0.0;
        public double intakeExtenderVelocity = 0.0;

        public double intakeRollerRightVoltage = 0.0;
        public double intakeRollerLeftVoltage = 0.0;
        public double intakeExtenderVoltage = 0.0;

        public boolean intakeRollerRightConnected = false;
        public boolean intakeRollerLeftConnected = false;
        public boolean intakeExtenderConnected = false;

        public String setpoint = "";
    }

    public void setSpeed(double speed);

    public void setVelocity(double velocity);

    public default void setExtenderSetpoint(double rotations) {}

    // Get Hardware Data to Business Logic
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs);

    public void setPosition(double position);
}
