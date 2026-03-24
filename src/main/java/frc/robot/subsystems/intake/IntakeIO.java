package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        // public double speed = 0.0;
        // public double statorCurrent = 0.0;
        // public double supplyCurrent = 0.0;
        // public double position = 0.0;
        // public double velocity = 0.0;

        public String setpoint = "";

        public boolean intakeRoller1Connected = false;
        public double intakeRoller1VelocityRPS = 0.0;
        public double intakeRoller1StatorCurrent = 0.0;
        public double intakeRoller1SupplyCurrent = 0.0;
        public double intakeRoller1Voltage = 0.0;

        public boolean intakeRoller2Connected = false;
        public double intakeRoller2VelocityRPS = 0.0;
        public double intakeRoller2StatorCurrent = 0.0;
        public double intakeRoller2SupplyCurrent = 0.0;
        public double intakeRoller2Voltage = 0.0;

        public boolean intakePivotConnected = false;
        public double intakePivotVelocityRPS = 0.0;
        public double intakePivotStatorCurrent = 0.0;
        public double intakePivotSupplyCurrent = 0.0;
        public double intakePivotVoltage = 0.0;
        public double intakePivotRotations = 0.0;

        public double goalPosition = 0.0;

        public double goalVelocity = 0.0;
    }

    public void setSpeed(double speed);

    public void setVelocity(double velocity);

    public default void setExtenderSetpoint(double rotations) {}

    // Get Hardware Data to Business Logic
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs);

    public void setPosition(double position);
}
