package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        super("Shooter");
        m_io = io;
    }

    public Command slowShoot() {
        return this.run(() -> m_io.setSpeed(-0.2)).withName("SlowShoot");
    }

    public Command shoot() {
        return this.run(() -> m_io.setSpeed(-1)).withName("Shoot");
    }

    public Command stop() {
        return this.run(() -> m_io.setSpeed(0)).withName("Stop");
    }

    public Command stopInstant() {
        return this.runOnce(() -> m_io.setSpeed(0)).withName("StopInstant");
    }

    public Command intake() {
        return this.run(() -> m_io.setSpeed(1)).withName("Intake");

    }

    @Override
    public void periodic() {
        Logger.processInputs(this.getName(), m_inputs);
    }
}
