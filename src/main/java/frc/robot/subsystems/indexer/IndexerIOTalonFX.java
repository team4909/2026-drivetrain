package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class IndexerIOTalonFX implements IndexerIO {

    // Bottom = leader
    private final TalonFX m_indexerBottomMotor;
    // Top = follower
    private final TalonFX m_indexerTopMotor;

    private final int kIndexerBottomMotorID = 54;
    private final int kIndexerTopMotorID = 53;

    private final String kCanbus = "CANivore2";

    private StatusSignal<AngularVelocity> m_spindexerVelocity;
    private StatusSignal<Current> m_spindexerStatorCurrent;
    private StatusSignal<Current> m_spindexerSupplyCurrent;
    private StatusSignal<Voltage> m_spindexerVoltage;

    private StatusSignal<AngularVelocity> m_kickerVelocity;
    private StatusSignal<Current> m_kickerStatorCurrent;
    private StatusSignal<Current> m_kickerSupplyCurrent;
    private StatusSignal<Voltage> m_kickerVoltage;

    private double m_goalVelocity;

    public IndexerIOTalonFX() {
        m_indexerBottomMotor = new TalonFX(kIndexerBottomMotorID, kCanbus);
        m_indexerTopMotor = new TalonFX(kIndexerTopMotorID, kCanbus);

        final TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_indexerBottomMotor.getConfigurator().apply(indexerMotorConfig);
        m_indexerTopMotor.getConfigurator().apply(indexerMotorConfig);

     
        // m_indexerTopMotor.setControl(
        //     new Follower(kIndexerBottomMotorID, MotorAlignmentValue.Opposed)
        // );

        m_spindexerVelocity = m_indexerBottomMotor.getVelocity();
        m_spindexerStatorCurrent = m_indexerBottomMotor.getStatorCurrent();
        m_spindexerSupplyCurrent = m_indexerBottomMotor.getSupplyCurrent();
        m_spindexerVoltage = m_indexerBottomMotor.getSupplyVoltage();

        m_kickerVelocity = m_indexerTopMotor.getVelocity();
        m_kickerStatorCurrent = m_indexerTopMotor.getStatorCurrent();
        m_kickerSupplyCurrent = m_indexerTopMotor.getSupplyCurrent();
        m_kickerVoltage = m_indexerTopMotor.getSupplyVoltage();

        m_goalVelocity = 0;
    }

    @Override
    public void setSpeed(double speed) {
        m_goalVelocity = speed;
        m_indexerBottomMotor.setControl(new DutyCycleOut(speed));
        m_indexerTopMotor.setControl(new DutyCycleOut(-speed*0.75));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        m_indexerBottomMotor.setNeutralMode(neutralModeValue);
        m_indexerTopMotor.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(IndexerIOInputsAutoLogged m_inputs) {
        m_inputs.spindexerConnected = 
            BaseStatusSignal.refreshAll(
                m_spindexerVelocity,
                m_spindexerVoltage,
                m_spindexerStatorCurrent,
                m_spindexerSupplyCurrent
            ).isOK();

        m_inputs.spindexerVelocityRPS = m_spindexerVelocity.getValueAsDouble();
        m_inputs.spindexerStatorCurrent = m_spindexerStatorCurrent.getValueAsDouble();
        m_inputs.spindexerSupplyCurrent = m_spindexerSupplyCurrent.getValueAsDouble();
        m_inputs.spindexerVoltage = m_spindexerVoltage.getValueAsDouble();

        m_inputs.kickerConnected = 
            BaseStatusSignal.refreshAll(
                m_kickerVelocity,
                m_kickerVoltage,
                m_kickerStatorCurrent,
                m_kickerSupplyCurrent
            ).isOK();
        m_inputs.kickerVelocityRPS = m_kickerVelocity.getValueAsDouble();
        m_inputs.kickerStatorCurrent = m_kickerStatorCurrent.getValueAsDouble();
        m_inputs.kickerSupplyCurrent = m_kickerSupplyCurrent.getValueAsDouble();
        m_inputs.kickerVoltage = m_kickerVoltage.getValueAsDouble();

        m_inputs.goalVelocity = m_goalVelocity;
    }
}
