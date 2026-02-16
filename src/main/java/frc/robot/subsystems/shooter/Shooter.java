package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
    private LoggedNetworkNumber m_velocity = new LoggedNetworkNumber("/Tuning/ShooterVelocityRPS", -50.0);
    private double m_targetVelocity = 0.0;
    private final double kSHOOTERTOLERANCE = 1;

    public Shooter(ShooterIO io) {
        super("Shooter");
        m_io = io;
    }

    public Command tuningShoot() {
        m_targetVelocity = m_velocity.get();
        return this.run(() -> m_io.setVelocity(m_velocity.get()));
    }

    public Command stop() {
        m_targetVelocity = 0.0;
        return this.run(() -> m_io.setVelocity(0)).withName("Stop");
    }

    private void setShooterVelocity(double velocity) {
        m_targetVelocity = velocity;
        m_io.setVelocity(velocity);
    }

    public Command shoot(DoubleSupplier velocity) {
        return this.run(() -> setShooterVelocity((int) velocity.getAsDouble())).repeatedly().withName("Shoot");
        // m_targetVelocity = velocity.getAsDouble();
        // return this.run(() -> m_io.setVelocity((int)
        // velocity.getAsDouble())).repeatedly().withName("Shoot");
    }

    public boolean atSpeed() {
        if (m_targetVelocity == 0.0) {
            return false;
        }

        return MathUtil.isNear(m_targetVelocity, m_inputs.motor1VelocityRPS, kSHOOTERTOLERANCE)
                && MathUtil.isNear(m_targetVelocity, m_inputs.motor2VelocityRPS, kSHOOTERTOLERANCE);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);

        Logger.recordOutput("Shooter/atSpeed", atSpeed());
    }
}
