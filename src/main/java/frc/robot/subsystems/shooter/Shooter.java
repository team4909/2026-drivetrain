package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
    private LoggedNetworkNumber m_velocity = new LoggedNetworkNumber("/Tuning/ShooterVelocityRPS", -50.0);

    public Shooter(ShooterIO io) {
        super("Shooter");
        m_io = io;
    }

    public Command tuningShoot() {
        return this.run(() -> m_io.setVelocity(m_velocity.get()));
    }

    public Command stop() {
        return this.run(() -> m_io.setVelocity(0)).withName("Stop");
    }

    public Command shoot(){
        return this.run(() -> m_io.setVelocity(-50.0)).withName("Shoot");
    }

    public Command shootFromDistance(DoubleSupplier distanceMeters, ShootingCalculator calculator) {
        return this.run(
                () -> {
                    double distance = distanceMeters.getAsDouble();
                    if (!Double.isFinite(distance)) {
                        m_io.setVelocity(0.0);
                        return;
                    }
                    m_io.setVelocity(calculator.getShooterSpeed(distance));
                })
            .withName("ShootFromTagDistance");
    }

    @Override
    public void periodic() {
        Logger.processInputs(this.getName(), m_inputs);
    }
}