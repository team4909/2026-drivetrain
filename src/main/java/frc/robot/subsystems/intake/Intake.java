package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;

public class Intake extends SubsystemBase {
    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
    private LoggedNetworkNumber m_position = new LoggedNetworkNumber("/Tuning/IntakePosition", 0);
    private LoggedNetworkNumber m_velocity = new LoggedNetworkNumber("/Tuning/IntakeVelocity", 0);

    public Intake(IntakeIO io) {
        super("Intake");
        m_io = io;

    }

    public Command run() {
        return this.run(() -> m_io.setSpeed(m_velocity.get())).withName("IntakeRun");
    }

    public Command stop() {
        return this.run(() -> m_io.setSpeed(0)).withName("IntakeStop");
    }

    public Command intake() {
        return this.run(() -> m_io.setSpeed(1)).withName("IntakeIn");
    }

    public Command outtake() {
        return this.run(() -> m_io.setSpeed(-1)).withName("IntakeOut");
    }

    public Command extend() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(1.0)).withName("IntakeExtend");
    }

    public Command retract() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(0)).withName("IntakeRetract");
    }

    @Override
    public void periodic() {
        Logger.processInputs(this.getName(), m_inputs);
    }
}
