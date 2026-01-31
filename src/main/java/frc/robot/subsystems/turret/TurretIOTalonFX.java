package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO{
    private final TalonFX m_motor;
    private final double m_gearBox = 1.0/3;
    private final double m_smallGear = 10;
    private final double m_bigGear = 105;
    private final double m_gearRatio = m_gearBox * (m_smallGear / m_bigGear);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private StatusSignal<AngularVelocity> m_velocity;
    private StatusSignal<Voltage> m_voltage;
    private StatusSignal m_current;
    private StatusSignal m_rotations;

    public TurretIOTalonFX (){
        m_motor = new TalonFX(60,"CANivore1");

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1d; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        final MotorOutputConfigs turretConfigs = new MotorOutputConfigs();
        turretConfigs.NeutralMode = NeutralModeValue.Brake;

        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = m_gearRatio;

        m_motor.getConfigurator().apply(slot0Configs);
        m_motor.getConfigurator().apply(feedbackConfigs);
        m_motor.getConfigurator().apply(turretConfigs);

        m_velocity = m_motor.getVelocity();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getStatorCurrent();
        m_rotations = m_motor.getPosition();
    }

    public void setSpeed(double speed) {
        m_motor.setControl(new DutyCycleOut(speed));
    }

    public void setSetpoint(double rotations) {
        m_motor.setControl(m_request.withPosition(rotations));
    }

    public void updateInputs(TurretIOInputsAutoLogged m_inputs){
        m_inputs.current = m_current.getValueAsDouble();
        m_inputs.rotations = m_rotations.getValueAsDouble();
        m_inputs.velocityRPS = m_velocity.getValueAsDouble();
        m_inputs.volts = m_voltage.getValueAsDouble();
    }
}
