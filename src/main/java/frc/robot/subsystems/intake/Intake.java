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
     private LoggedNetworkNumber m_home = new LoggedNetworkNumber("/Tuning/IntakePosition", 0);
    private LoggedNetworkNumber m_upPosition = new LoggedNetworkNumber("/Tuning/Intake/UpPosition", 0.0);
    private LoggedNetworkNumber m_downPosition = new LoggedNetworkNumber("/Tuning/Intake/DownPosition", 10.0);
    private LoggedNetworkNumber m_forceDownVolts = new LoggedNetworkNumber("/Tuning/Intake/ForceDownVolts", 2.0);
    private LoggedNetworkNumber m_oscillateWait = new LoggedNetworkNumber("/Tuning/Intake/OscillateWait", 0.3);

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
        return this.run(() -> m_io.setSpeed(1)).withName("IntakeOut");
    }

    public Command goToExtender() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(m_position.get())).withName("GoToExtender");
    }

    public Command goTohome() {
        return this.runOnce(() -> m_io.setExtenderSetpoint(m_home.get())).withName("GoToExtender");
    }

    private Command setPosition(double rotations) {
        return this.runOnce(() -> m_io.setExtenderSetpoint(rotations));
    }

    public Command oscillateExtender() {
        return Commands.sequence(
                setPosition(m_upPosition.get()),
                Commands.waitSeconds(0.3), // Wait to reach up
                Commands.sequence(
                        setPosition(m_downPosition.get()),
                        Commands.waitSeconds(0.3), // Wait to reach down
                        setPosition(m_upPosition.get()),
                        Commands.waitSeconds(0.3)).repeatedly())
                .withName("OscillateExtender");
    }

    @Override
    public void periodic() {
        Logger.processInputs(this.getName(), m_inputs);
    }
}
