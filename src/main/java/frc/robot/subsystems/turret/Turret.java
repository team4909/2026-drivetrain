package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private final TurretIO m_io;
    

    public Turret (TurretIO io) {
        m_io = io;
    }

    public Command go() {
        return this.run(() -> m_io.setSpeed(0.1));
    }

    public Command stop() {
        return this.run(() -> m_io.setSpeed(0));
    }

    public Command home() {
        return this.run(() -> m_io.setSetpoint(3));
    }

    public Command goToAngle(double degrees) {
        return this.run(() -> m_io.setSetpoint(degrees));
    }
}
