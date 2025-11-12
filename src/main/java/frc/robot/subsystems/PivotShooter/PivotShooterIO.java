package frc.robot.subsystems.PivotShooter;

import org.littletonrobotics.junction.AutoLog;

public interface PivotShooterIO {
    
    @AutoLog
    public static class PivotShooterIOInputs { 
        public double shooterVoltage = 0d;
        public double shooterCurrent = 0;
        public double shootVelocity = 0;
        public double wristPosition = 0;
        public double wristSetpoint = 0;
    }

    public default void setShootVoltage(double voltage) {}

    public default void setPivotVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

	public void updateInputs(PivotShooterIOInputs m_inputs);

	public void holdShooterPos();
}
