package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class IndexerIOTalonFX implements IndexerIO {

    // Bottom = leader
    private final TalonFX m_indexerBottomMotor;
    // Top = follower
    private final TalonFX m_indexerTopMotor;

    private final int kIndexerBottomMotorID = 22;
    private final int kIndexerTopMotorID = 23;

    private final String kCanbus = "CANivore2";

    public IndexerIOTalonFX() {
        m_indexerBottomMotor = new TalonFX(kIndexerBottomMotorID, kCanbus);
        m_indexerTopMotor = new TalonFX(kIndexerTopMotorID, kCanbus);

        final TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_indexerBottomMotor.getConfigurator().apply(indexerMotorConfig);
        m_indexerTopMotor.getConfigurator().apply(indexerMotorConfig);

     
        m_indexerTopMotor.setControl(
            new Follower(kIndexerBottomMotorID, MotorAlignmentValue.Aligned)
        );
    }

    @Override
    public void setSpeed(double speed) {
  
        m_indexerBottomMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        m_indexerBottomMotor.setNeutralMode(neutralModeValue);
        m_indexerTopMotor.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(IndexerIOInputsAutoLogged m_inputs) {
        m_inputs.speed = m_indexerBottomMotor.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_indexerBottomMotor.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_indexerBottomMotor.getSupplyCurrent().getValueAsDouble();
    }
}
