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
        m_inputs.command = "TuningShoot";
        return this.run(() -> m_io.setSpeed(m_speed.get()));
    }

    public Command slowFeed() {
        m_inputs.command = "SlowShoot";
        return this.run(() -> m_io.setSpeed(-0.2)).withName("SlowShoot");
    }

    public Command feed() {
        //log feed
        m_inputs.command = "Feed";
        return this.run(() -> m_io.setSpeed(1)).withName("Shoot");
    }

    public Command stop() {
        //log stop
        m_inputs.command = "Stop";
        return this.run(() -> m_io.setSpeed(0)).withName("Stop");
    }

    public Command stopInstant() {
        m_inputs.command = "StopInstant";
        return this.runOnce(() -> m_io.setSpeed(0)).withName("StopInstant");
    }

    public Command notintake() {
        m_inputs.command = "NotIntake";
        return this.run(() -> m_io.setSpeed(-1)).withName("NotIntake");

    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);
    }
}
