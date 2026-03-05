package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Turret extends SubsystemBase{

    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();
    private double turretDegrees = 0;
    // private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

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

    public Command goToAngle(double degrees) {
        return this.run(() -> m_io.setSetpoint(degrees));
    }
    /** Immediately set turret position in degrees (0 = forward). */
    public void setDegrees(double degrees) {
        m_io.setSetpoint(degrees / 360.0);
    }

    public double getTurretPosition() {return m_io.getTurretPosition();}

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(getName(), m_inputs);
    }
}
