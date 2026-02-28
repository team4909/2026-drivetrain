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
    private final double Stowed = 0.0;
    private final double Extended = -7;


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
        return this.run(() -> m_io.setSpeed(-1)).withName("IntakeIn");
    }

    public Command intakeVelocity() {
        return this.run(() -> m_io.setVelocity(10.0)).withName("IntakeInVelocity");
    }

    public Command outtake() {
        return this.run(() -> m_io.setSpeed(-1)).withName("IntakeOut");
    }

    public Command intakeAndExtend() {
        return this.run(() -> {
            m_io.setExtenderSetpoint(Extended);
            m_io.setSpeed(-1);
            m_inputs.setpoint = "Extend";
        }).withName("IntakeWithSetpoint").until(() -> Math.abs(m_inputs.position - Extended) <= 0.1);
    }

     public Command Extend() {
        return this.run(() -> {
            m_io.setExtenderSetpoint(Extended);
            // m_io.setSpeed(-1);
            m_inputs.setpoint = "Extend";
        }).withName("IntakeWithSetpoint").until(() -> Math.abs(m_inputs.position - Extended) <= 0.1);
    }

    public Command stowAndStop() {
        return this.run(() -> {
            m_io.setExtenderSetpoint(Stowed);
            m_io.setSpeed(0);
            m_inputs.setpoint = "Stowed";
        }).withName("IntakeStowAndStop").until(() -> Math.abs(m_inputs.position - Stowed) <= 0.1);
    }
    public Command stow() {
        return this.run(() -> {
            m_io.setExtenderSetpoint(Stowed);
            // m_io.setSpeed(0);
            m_inputs.setpoint = "Stowed";
        }).withName("IntakeStowAndStop").until(() -> Math.abs(m_inputs.position - Stowed) <= 0.1);
    }

    public Command setpointFromTuning() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(m_position.get()))
                .withName("IntakeSetpointTuning");
    }

    public Command reZero() {
        return this.runOnce(() ->{
            m_io.setPosition(0);
        });
    }

    public Command reZeroDown() {
        return this.runOnce(() ->{
            m_io.setPosition(10);
        });
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);
    }
}