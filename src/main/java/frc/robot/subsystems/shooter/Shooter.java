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
    private double kSHOOTERTOLERANCE = 5;


    public Shooter(ShooterIO io) {
        super("Shooter");
        m_io = io;
    }

    public Command tuningShoot() {
        return this.run(() -> m_io.setVelocity(m_velocity.get()));
    }

    public Command stop() {
        return this.run(() -> m_io.setDutyCycle(0)).withName("Stop");
    }

    public Command shoot(DoubleSupplier velocity){
        return this.run(() -> m_io.setVelocity(velocity.getAsDouble())).repeatedly().withName("Shoot");
    }

    public boolean atSpeed() {
        if (m_inputs.goalVelocity == 0.0 || m_inputs.goalVelocity == 30.0) {
            return false;
        }

        return MathUtil.isNear(m_inputs.goalVelocity, m_inputs.motor1VelocityRPS, kSHOOTERTOLERANCE)
                || MathUtil.isNear(m_inputs.goalVelocity, m_inputs.motor2VelocityRPS, kSHOOTERTOLERANCE);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(this.getName(), m_inputs);

        Logger.recordOutput("Shooter/AtSpeed", atSpeed());
    }
}
