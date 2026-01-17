package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    
    @AutoLog
    public static class TurretIOInputs { 
        public double velocityRPS = 0.0;
        public double volts = 0.0;
        public double current = 0.0;
        public double rotations;
    }

    public default void setSpeed(double speed){}

    public default void setSetpoint(double rotations){}
}