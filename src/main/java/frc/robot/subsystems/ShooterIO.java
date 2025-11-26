package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double speed = 0.0;
        public double statorCurrent = 0.0;
        public double supplyCurrent = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setSpeed(double speed);

    public void setBrakeMode(boolean enableBrakeMode);

    // Get Hardware Data to Business Logic
    public void updateInputs(ShooterIOInputsAutoLogged m_inputs);
}