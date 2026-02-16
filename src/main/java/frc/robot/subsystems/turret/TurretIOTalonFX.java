package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX m_motor;
    private final double m_gearBox = 1.0 / 3;
    private final double m_smallGear = 10;
    private final double m_bigGear = 105;
    private final double m_gearRatio = (m_gearBox * (m_smallGear / m_bigGear));
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private final int kShooterMotorID = 60;
    private final String kCanbus = "CANivore1";

    private StatusSignal<AngularVelocity> m_velocity;
    private StatusSignal<Voltage> m_voltage;
    private StatusSignal<Current> m_supplyCurrent;
    private StatusSignal<Angle> m_rotations;

    public TurretIOTalonFX() {
        m_motor = new TalonFX(kShooterMotorID, kCanbus);

        var TalonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = TalonFXConfigs.Slot0;
        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;
        slot0Configs.kP = 20;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        var motionMagicConfigs = TalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        motionMagicConfigs.MotionMagicAcceleration = 160;

        TalonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;
        TalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        final MotorOutputConfigs turretConfiguration = new MotorOutputConfigs();
        turretConfiguration.NeutralMode = NeutralModeValue.Brake;

        m_motor.getConfigurator().apply(TalonFXConfigs);
        m_motor.getConfigurator().apply(turretConfiguration);

        m_velocity = m_motor.getVelocity();
        m_voltage = m_motor.getMotorVoltage();
        m_supplyCurrent = m_motor.getStatorCurrent();
        m_rotations = m_motor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
                m_velocity,
                m_voltage,
                m_supplyCurrent,
                m_rotations);

        m_motor.setPosition(0, 2);
    }

    public void setSpeed(double speed) {
        m_motor.setControl(new DutyCycleOut(speed));
    }

    public void setSetpoint(double rotations) {
        m_motor.setControl(m_request.withPosition(Rotations.of(rotations / m_gearRatio)));
    }

    public void updateInputs(TurretIOInputsAutoLogged m_inputs) {
        m_inputs.motorConnected = 
        BaseStatusSignal.refreshAll(
            m_velocity,
            m_voltage,
            m_supplyCurrent,
            m_rotations
        ).isOK();

        m_inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();
        m_inputs.rotations = m_rotations.getValueAsDouble();
        m_inputs.velocityRPS = m_velocity.getValueAsDouble();
        m_inputs.volts = m_voltage.getValueAsDouble();
    }

    public double getTurretPosition() {
        return m_motor.getPosition().getValueAsDouble() * m_gearRatio;
    }
}
