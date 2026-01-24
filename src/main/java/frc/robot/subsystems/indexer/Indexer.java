package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class Indexer extends SubsystemBase {
    private final IndexerIO m_io;
    private final IndexerIOInputsAutoLogged m_inputs = new IndexerIOInputsAutoLogged();
    private LoggedNetworkNumber m_speed;

    public Indexer(IndexerIO io) {
        super("Indexer");
        m_io = io;

        m_speed = new LoggedNetworkNumber("/Tuning/IndexerSpeed", 0.0);
    }

    public Command tuningShoot() {
        return this.run(() -> m_io.setSpeed(m_speed.get()));
    }

    public Command slowFeed() {
        return this.run(() -> m_io.setSpeed(-0.2)).withName("SlowShoot");
    }

    public Command feed() {
        return this.run(() -> m_io.setSpeed(1)).withName("Shoot");
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
