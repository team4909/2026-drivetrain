package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public interface TurretIO {
    
    @AutoLog
    public static class TurretIOInputs {
        public boolean motorConnected = false;
        public double velocityRPS = 0.0;
        public double volts = 0.0;
        public double supplyCurrent = 0.0;
        public double rotations = 0.0;
    }

    public default void setSpeed(double speed){}

    public default void setSetpoint(double rotations){}

    public default double getTurretPosition(){return 0.0;}

    public void updateInputs(TurretIOInputsAutoLogged m_inputs);
}