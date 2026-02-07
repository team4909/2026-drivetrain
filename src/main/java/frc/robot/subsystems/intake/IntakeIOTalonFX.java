package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.controls.VoltageOut;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX m_intakeRoller;
    private final TalonFX m_intakeExtender;
    private final TalonFX m_intakeRollerLeft;
    private PositionVoltage m_extenderRequest;
    private VoltageOut m_voltageRequest;

    private final int kIntakeRollerID = 56;
    private final int kIntakeRollerLeaderID = 57;
    private final int kIntakeExtenderID = 55;

    private final String kCanbus = "CANivore2";

    public IntakeIOTalonFX() {
        m_intakeRollerLeft = new TalonFX(kIntakeRollerLeaderID, kCanbus);
        m_intakeRoller = new TalonFX(kIntakeRollerID, kCanbus);
        m_intakeExtender = new TalonFX(kIntakeExtenderID, kCanbus);

        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Slot0.kP = 1.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        m_intakeRollerLeft.getConfigurator().apply(cfg);
        m_intakeExtender.getConfigurator().apply(cfg);
        m_intakeRoller.getConfigurator().apply(cfg);

        m_intakeRoller.setControl(new Follower(kIntakeRollerLeaderID, MotorAlignmentValue.Opposed));

        m_extenderRequest = new PositionVoltage(0).withSlot(0);
        m_voltageRequest = new VoltageOut(0);

    }

    @Override
    public void setSpeed(double speed) {
        m_intakeRollerLeft.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_intakeRoller.setNeutralMode(neutralModeValue);
        m_intakeExtender.setNeutralMode(neutralModeValue);
    }

    @Override
    public void setExtenderSetpoint(double rotations) {
        m_intakeExtender.setControl(m_extenderRequest.withPosition(rotations));
    }

    @Override
    public void setExtenderVoltage(double volts) {
        m_intakeExtender.setControl(m_voltageRequest.withOutput(volts));
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs) {
        m_inputs.speed = m_intakeRoller.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_intakeRoller.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_intakeRoller.getSupplyCurrent().getValueAsDouble();
    }

}
