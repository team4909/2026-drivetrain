package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerIOInputsAutoLogged;

public class IndexerIOTalonFX implements IndexerIO {


    
    // // Bottom = leader
    // private final TalonFX m_indexerBottomMotor;
    // // Top = follower
    // private final TalonFX m_indexerTopMotor;

    // private final int kIndexerBottomMotorID = 54;
    // private final int kIndexerTopMotorID = 53;

    // private final String kCanbus = "CANivore2";

    // private StatusSignal<AngularVelocity> m_spindexerVelocity;
    // private StatusSignal<Current> m_spindexerStatorCurrent;
    // private StatusSignal<Current> m_spindexerSupplyCurrent;
    // private StatusSignal<Voltage> m_spindexerVoltage;

    // private StatusSignal<AngularVelocity> m_kickerVelocity;
    // private StatusSignal<Current> m_kickerStatorCurrent;
    // private StatusSignal<Current> m_kickerSupplyCurrent;
    // private StatusSignal<Voltage> m_kickerVoltage;

    // private double m_goalVelocity;

    // public IndexerIOTalonFX() {
    //     m_indexerBottomMotor = new TalonFX(kIndexerBottomMotorID, kCanbus);
    //     m_indexerTopMotor = new TalonFX(kIndexerTopMotorID, kCanbus);

    //     final TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
    //     indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    //     indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    //     indexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //     indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //     m_indexerBottomMotor.getConfigurator().apply(indexerMotorConfig);
    //     m_indexerTopMotor.getConfigurator().apply(indexerMotorConfig);

     
    //     m_indexerTopMotor.setControl(
    //         new Follower(kIndexerBottomMotorID, MotorAlignmentValue.Opposed)
    //     );

    //     m_spindexerVelocity = m_indexerBottomMotor.getVelocity();
    //     m_spindexerStatorCurrent = m_indexerBottomMotor.getStatorCurrent();
    //     m_spindexerSupplyCurrent = m_indexerBottomMotor.getSupplyCurrent();
    //     m_spindexerVoltage = m_indexerBottomMotor.getSupplyVoltage();

    //     m_kickerVelocity = m_indexerTopMotor.getVelocity();
    //     m_kickerStatorCurrent = m_indexerTopMotor.getStatorCurrent();
    //     m_kickerSupplyCurrent = m_indexerTopMotor.getSupplyCurrent();
    //     m_kickerVoltage = m_indexerTopMotor.getSupplyVoltage();

    //     m_goalVelocity = 0;
    // }

    // @Override
    // public void setVelocity(double speed) {
    //     m_goalVelocity = speed;
    //     m_indexerBottomMotor.setControl(new DutyCycleOut(speed));
    // }

    // @Override
    // public void setBrakeMode(boolean enableBrakeMode) {
    //     final NeutralModeValue neutralModeValue =
    //         enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    //     m_indexerBottomMotor.setNeutralMode(neutralModeValue);
    //     m_indexerTopMotor.setNeutralMode(neutralModeValue);
    // }

    // public void updateInputs(IndexerIOInputsAutoLogged m_inputs) {
    //     // m_inputs.spindexerConnected = 
    //     //     BaseStatusSignal.refreshAll(
    //     //         m_spindexerVelocity,
    //     //         m_spindexerVoltage,
    //     //         m_spindexerStatorCurrent,
    //     //         m_spindexerSupplyCurrent
    //     //     ).isOK();

    //     // m_inputs.spindexerVelocityRPS = m_spindexerVelocity.getValueAsDouble();
    //     // m_inputs.spindexerStatorCurrent = m_spindexerStatorCurrent.getValueAsDouble();
    //     // m_inputs.spindexerSupplyCurrent = m_spindexerSupplyCurrent.getValueAsDouble();
    //     // m_inputs.spindexerVoltage = m_spindexerVoltage.getValueAsDouble();

    //     // m_inputs.kickerConnected = 
    //     //     BaseStatusSignal.refreshAll(
    //     //         m_kickerVelocity,
    //     //         m_kickerVoltage,
    //     //         m_kickerStatorCurrent,
    //     //         m_kickerSupplyCurrent
    //     //     ).isOK();
    //     // m_inputs.kickerVelocityRPS = m_kickerVelocity.getValueAsDouble();
    //     // m_inputs.kickerStatorCurrent = m_kickerStatorCurrent.getValueAsDouble();
    //     // m_inputs.kickerSupplyCurrent = m_kickerSupplyCurrent.getValueAsDouble();
    //     // m_inputs.kickerVoltage = m_kickerVoltage.getValueAsDouble();

    //     m_inputs.goalVelocity = m_goalVelocity;
    // }





















    // Bottom = leader
    private final TalonFX m_rollerFloor1Motor;
    // Top = follower
    private final TalonFX m_rollerFloor2Motor;

    private final TalonFX m_kickerMotor;

    // private final TalonFX m_rollerFloor4Motor;


    private final int m_rollerFloor1MotorID = 53;
    private final int m_rollerFloor2MotorID = 54;
    private final int m_kickerMotorID = 52;


    private final String kCanbus = "CANivore2";

    private StatusSignal<AngularVelocity> m_rollerFloor1Velocity;
    private StatusSignal<Current> m_rollerFloor1StatorCurrent;
    private StatusSignal<Current> m_rollerFloor1SupplyCurrent;
    private StatusSignal<Voltage> m_rollerFloor1Voltage;

    private StatusSignal<AngularVelocity> m_rollerFloor2Velocity;
    private StatusSignal<Current> m_rollerFloor2StatorCurrent;
    private StatusSignal<Current> m_rollerFloor2SupplyCurrent;
    private StatusSignal<Voltage> m_rollerFloor2Voltage;

    private StatusSignal<AngularVelocity> m_kickerVelocity;
    private StatusSignal<Current> m_kickerStatorCurrent;
    private StatusSignal<Current> m_kickerSupplyCurrent;
    private StatusSignal<Voltage> m_kickerVoltage;

    // private StatusSignal<AngularVelocity> m_rollerFloor4Velocity;
    // private StatusSignal<Current> m_rollerFloor4StatorCurrent;
    // private StatusSignal<Current> m_rollerFloor4SupplyCurrent;
    // private StatusSignal<Voltage> m_rollerFloor4Voltage;

    private VelocityTorqueCurrentFOC m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private double m_goalVelocity;

    public IndexerIOTalonFX() {
        m_rollerFloor1Motor = new TalonFX(m_rollerFloor1MotorID, kCanbus);
        m_rollerFloor2Motor = new TalonFX(m_rollerFloor2MotorID, kCanbus);
        m_kickerMotor = new TalonFX(m_kickerMotorID, kCanbus);
        
        final TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();
        // indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        // indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotorConfig.Slot0.kP = 7;
        indexerMotorConfig.Slot0.kI = 0;
        indexerMotorConfig.Slot0.kD = 0.1;
        indexerMotorConfig.Slot0.kS = 0;
        indexerMotorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(Units.Amps.of(40)).withPeakReverseTorqueCurrent(Units.Amps.of(-40));

        m_rollerFloor1Motor.getConfigurator().apply(indexerMotorConfig);
        m_rollerFloor2Motor.getConfigurator().apply(indexerMotorConfig);
        m_kickerMotor.getConfigurator().apply(indexerMotorConfig);

     
        // m_rollerFloor2Motor.setControl(
        //     new Follower(m_rollerFloor1MotorID, MotorAlignmentValue.Opposed)
        // );

        //  m_rollerFloor3Motor.setControl(
        //     new Follower(m_rollerFloor1MotorID, MotorAlignmentValue.Opposed)
        // );

        m_rollerFloor1Velocity = m_rollerFloor1Motor.getVelocity();
        m_rollerFloor1StatorCurrent = m_rollerFloor1Motor.getStatorCurrent();
        m_rollerFloor1SupplyCurrent = m_rollerFloor1Motor.getSupplyCurrent();
        m_rollerFloor1Voltage = m_rollerFloor1Motor.getSupplyVoltage();

        m_rollerFloor2Velocity = m_rollerFloor2Motor.getVelocity();
        m_rollerFloor2StatorCurrent = m_rollerFloor2Motor.getStatorCurrent();
        m_rollerFloor2SupplyCurrent = m_rollerFloor2Motor.getSupplyCurrent();
        m_rollerFloor2Voltage = m_rollerFloor2Motor.getSupplyVoltage();

        m_kickerVelocity = m_kickerMotor.getVelocity();
        m_kickerStatorCurrent = m_kickerMotor.getStatorCurrent();
        m_kickerSupplyCurrent = m_kickerMotor.getSupplyCurrent();
        m_kickerVoltage = m_kickerMotor.getSupplyVoltage();

    
        m_goalVelocity = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
            m_rollerFloor1Velocity,
            m_rollerFloor1StatorCurrent,
            m_rollerFloor1SupplyCurrent,
            m_rollerFloor1Voltage,
            m_rollerFloor2Velocity,
            m_rollerFloor2StatorCurrent,
            m_rollerFloor2SupplyCurrent,
            m_rollerFloor2Voltage,
            m_kickerVelocity,
            m_kickerStatorCurrent,
            m_kickerSupplyCurrent,
            m_kickerVoltage
          
        );
    }
    @Override
    public void setVelocity(double RPS) {
        m_goalVelocity = RPS;
        m_rollerFloor1Motor.setControl(m_velocityRequest.withVelocity(Units.RotationsPerSecond.of(-RPS)));
        m_rollerFloor2Motor.setControl(m_velocityRequest.withVelocity(Units.RotationsPerSecond.of(-RPS)));
        m_kickerMotor.setControl(m_velocityRequest.withVelocity(Units.RotationsPerSecond.of(-RPS)));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goalVelocity = dutyCycle;
        m_rollerFloor1Motor.setControl(new DutyCycleOut(dutyCycle));
        m_rollerFloor2Motor.setControl(new DutyCycleOut(dutyCycle));
        m_kickerMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    @Override
    public void setVelocityKicker(double RPS) {
        m_goalVelocity = RPS;
        m_kickerMotor.setControl(m_velocityRequest.withVelocity(Units.RotationsPerSecond.of(-RPS)));
    }
    

    // @Override
    // public void setSpeed(double speed) {
    //     // m_goalVelocity = speed;
    //    m_rollerFloor1Motor.setControl(new DutyCycleOut(speed));
    //    m_rollerFloor2Motor.setControl(new DutyCycleOut(speed));
    //    m_rollerFloor3Motor.setControl(new DutyCycleOut(speed));
    // }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        m_rollerFloor1Motor.setNeutralMode(neutralModeValue);
        m_rollerFloor2Motor.setNeutralMode(neutralModeValue);
        m_kickerMotor.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(IndexerIOInputsAutoLogged m_inputs) {
        m_inputs.rollerFloor1Connected = 
            BaseStatusSignal.refreshAll(
                m_rollerFloor1Velocity,
                m_rollerFloor1Voltage,
                m_rollerFloor1StatorCurrent,
                m_rollerFloor1SupplyCurrent
            ).isOK();

        m_inputs.rollerFloor1VelocityRPS = -m_rollerFloor1Velocity.getValueAsDouble();
        m_inputs.rollerFloor1StatorCurrent = m_rollerFloor1StatorCurrent.getValueAsDouble();
        m_inputs.rollerFloor1SupplyCurrent = m_rollerFloor1SupplyCurrent.getValueAsDouble();
        m_inputs.rollerFloor1Voltage = m_rollerFloor1Voltage.getValueAsDouble();

        m_inputs.rollerFloor2Connected = 
            BaseStatusSignal.refreshAll(
                m_rollerFloor2Velocity,
                m_rollerFloor2Voltage,
                m_rollerFloor2StatorCurrent,
                m_rollerFloor2SupplyCurrent
            ).isOK();
        m_inputs.rollerFloor2VelocityRPS = -m_rollerFloor2Velocity.getValueAsDouble();
        m_inputs.rollerFloor2StatorCurrent = m_rollerFloor2StatorCurrent.getValueAsDouble();
        m_inputs.rollerFloor2SupplyCurrent = m_rollerFloor2SupplyCurrent.getValueAsDouble();
        m_inputs.rollerFloor2Voltage = m_rollerFloor2Voltage.getValueAsDouble();

         m_inputs.kickerConnected = 
            BaseStatusSignal.refreshAll(
                m_kickerVelocity,
                m_kickerVoltage,
                m_kickerStatorCurrent,
                m_kickerSupplyCurrent
            ).isOK();
        m_inputs.kickerVelocityRPS = -m_kickerVelocity.getValueAsDouble();
        m_inputs.kickerStatorCurrent = m_kickerStatorCurrent.getValueAsDouble();
        m_inputs.kickerSupplyCurrent = m_kickerSupplyCurrent.getValueAsDouble();
        m_inputs.kickerVoltage = m_kickerVoltage.getValueAsDouble();


        m_inputs.goalVelocity = m_goalVelocity;
    }
}
