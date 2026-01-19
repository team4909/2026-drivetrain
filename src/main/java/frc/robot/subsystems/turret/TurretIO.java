package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    
    @AutoLog
    public static class TurretIOInputs { 
        public double speed = 0.0;
    }

    public default void setSpeed(double speed){}

    public default void setSetpoint(double rotations){}

    public default double getTurretPosition(){return 0.0;}
}