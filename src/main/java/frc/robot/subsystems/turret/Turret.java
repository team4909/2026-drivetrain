package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private final TurretIO m_io;
    private double turretDegrees = 0;

    public Turret (TurretIO io) {
        m_io = io;
    }

    public Command go() {
        return this.run(() -> m_io.setSpeed(0.1));
    }

    public Command stop() {
        return this.run(() -> m_io.setSpeed(0));
    }

    public Command goToDegrees(double degrees) {
        //0 Degrees is forwards
        return this.run(() -> m_io.setSetpoint(degrees/360));
    }

    /** Immediately set turret position in degrees (0 = forward). */
    public void setDegrees(double degrees) {
        m_io.setSetpoint(degrees / 360.0);
    }

    public double getTurretPosition() {return m_io.getTurretPosition();}
}
