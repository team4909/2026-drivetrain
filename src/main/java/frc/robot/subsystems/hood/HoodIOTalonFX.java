package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX m_hoodMotor;
    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);

    private static final int kHoodMotorID = 59;
    private static final String kCanbus = "CANivore2";

    private static final double kMinPulseWidth = 1000.0;
    private static final double kMaxPulseWidth = 2000.0;

    // Convert legacy pulse-width setpoints (1000-2000) into Falcon rotations.
    private static final double kMinRotations = 0.0;
    private static final double kMaxRotations = 0.35;

    public HoodIOTalonFX() {
        m_hoodMotor = new TalonFX(kHoodMotorID, kCanbus);

        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.5;

        final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        m_hoodMotor.getConfigurator().apply(config);
        m_hoodMotor.getConfigurator().apply(motorOutputConfigs);
    }

    @Override
    public void setPosition(double position) {
        final double clampedPulseWidth = MathUtil.clamp(position, kMinPulseWidth, kMaxPulseWidth);
        final double targetRotations = MathUtil.interpolate(
            kMinRotations,
            kMaxRotations,
            (clampedPulseWidth - kMinPulseWidth) / (kMaxPulseWidth - kMinPulseWidth)
        );

        m_hoodMotor.setControl(m_positionRequest.withPosition(targetRotations));
    }

    @Override
    public void updateInputs(HoodIOInputsAutoLogged inputs) {
        final double motorPosition = m_hoodMotor.getPosition().getValueAsDouble();
        inputs.positionLeftActuator = motorPosition;
        inputs.positionRightActuator = motorPosition;
    }
}