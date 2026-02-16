package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private final TurretIO m_io;
    private final TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

    public Turret (TurretIO io) {
        super("Turret");
        m_io = io;
    }

    public Command goToDegrees(double degrees) {
        //0 Degrees is forwards
        return this.run(() -> m_io.setSetpoint(degrees/360.0));
    }

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
