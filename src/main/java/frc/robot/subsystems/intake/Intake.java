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

        public enum Setpoint {
        Stowed(0.0),
        Extended(11.0);

        private final double rotations;

        Setpoint(double rotations) {
            this.rotations = rotations;
        }

        public double rotations() {
            return rotations;
        }
    }
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


     public Command setpoint(Setpoint setpoint) {
        return this.run(() -> m_io.setExtenderSetpoint(setpoint.rotations()))
                .withName("IntakeSetpoint" + setpoint.name()).until(() -> Math.abs(m_inputs.position - setpoint.rotations()) <= 0.1);
    }

    public Command holdSetpoint(Setpoint setpoint) {
        return this.run(() -> m_io.setExtenderSetpoint(setpoint.rotations()))
                .withName("IntakeHoldSetpoint" + setpoint.name());
    }

    public Command intakeWithSetpoint(Setpoint setpoint) {
        return this.run(() -> {
            m_io.setExtenderSetpoint(setpoint.rotations());
            m_io.setSpeed(1);
        }).withName("IntakeWithSetpoint" + setpoint.name());
    }

    
    
    // public Command stowAndStop() {
    //     return this.run(() -> {
    //         m_io.setExtenderSetpoint(getSetpointRotations(Setpoint.Stowed));
    //         m_io.setSpeed(0);
    //     }).withName("IntakeStowAndStop");
    // }
  

    public Command setpointFromTuning() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(m_position.get()))
                .withName("IntakeSetpointTuning");
    }

    public Command extend() {
        return setpoint(Setpoint.Extended).withName("IntakeExtend");
    }

    public Command retract() {
        return setpoint(Setpoint.Stowed).withName("IntakeRetract");
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);
    }
}