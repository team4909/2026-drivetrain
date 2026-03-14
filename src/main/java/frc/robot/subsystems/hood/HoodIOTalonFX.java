package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycle;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX m_hoodMotor;
    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);

    private static final int kHoodMotorID = 59;
    private static final String kCanbus = "CANivore2";

    private static final double kMinPulseWidth = 1000.0;
    private static final double kMaxPulseWidth = 2000.0;

    // Convert legacy pulse-width setpoints (1000-2000) into Falcon rotations.
    private static final double kMinRotations = 0.0;
    private static final double kMaxRotations = 0.685;

    public HoodIOTalonFX() {
        m_hoodMotor = new TalonFX(kHoodMotorID, kCanbus);

        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 25;
        config.Slot0.kI = 6;
        config.Slot0.kD = 0.01;
        config.Slot0.kS = 2.0;

        m_hoodMotor.getConfigurator().apply(config);

        m_hoodMotor.setPosition(0);
    }

    public double map(double x) {
        return (x - kMinPulseWidth) * (kMaxRotations - kMinRotations) / (kMaxPulseWidth - kMinPulseWidth) + kMinRotations;
    }

    @Override
    public void setPosition(double position) {
        // final double clampedPulseWidth = MathUtil.clamp(position, kMinPulseWidth, kMaxPulseWidth);
        // final double targetRotations = MathUtil.interpolate(
        //     kMinRotations,
        //     kMaxRotations,
        //     (clampedPulseWidth - kMinPulseWidth) / (kMaxPulseWidth - kMinPulseWidth)
        // );

        m_hoodMotor.setControl(m_positionRequest.withPosition( -MathUtil.clamp(position, kMinRotations, kMaxRotations)));
        Logger.recordOutput("Hood/map", -MathUtil.clamp(position, kMinRotations, kMaxRotations));
    }
    
    public void resetMotorPosition(double rotations){
        m_hoodMotor.setPosition(Rotations.of(rotations));
    }

    public void setSpeed(double speed) {
        //negative position/speed is hood up.
        m_hoodMotor.setControl(new DutyCycleOut(speed));
    }



    @Override
    public void updateInputs(HoodIOInputsAutoLogged inputs) {
        final double motorPosition = m_hoodMotor.getPosition().getValueAsDouble();
        inputs.positionLeftActuator = motorPosition;
        inputs.positionRightActuator = motorPosition;
    }
}