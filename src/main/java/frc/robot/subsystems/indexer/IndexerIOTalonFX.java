package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class IndexerIOTalonFX implements IndexerIO {

    private final TalonFX m_indexermotor;
    private final int kIndexerMotorID = 22;
    private final String kCanbus = "CANivore2";


    public IndexerIOTalonFX() {
        m_indexermotor = new TalonFX(kIndexerMotorID, kCanbus);

        final TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_indexermotor.getConfigurator().apply(indexerMotorConfig);
    }

    @Override
    public void setSpeed(double speed) {
        m_indexermotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_indexermotor.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(IndexerIOInputsAutoLogged m_inputs) {
        m_inputs.speed = m_indexermotor.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_indexermotor.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_indexermotor.getSupplyCurrent().getValueAsDouble();
    }

}