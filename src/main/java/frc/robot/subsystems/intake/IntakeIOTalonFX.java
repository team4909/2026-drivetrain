package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReasonValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX m_intakeRoller;
    private final TalonFX m_intakeExtender;
    private final TalonFX m_intakeRollerLeft;
    private PositionVoltage m_extenderRequest;
    // private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    private StatusSignal<AngularVelocity> m_intakeRoller1Velocity;
    private StatusSignal<Current> m_intakeRoller1StatorCurrent;
    private StatusSignal<Current> m_intakeRoller1SupplyCurrent;
    private StatusSignal<Voltage> m_intakeRoller1Voltage;

    private StatusSignal<AngularVelocity> m_intakeRoller2Velocity;
    private StatusSignal<Current> m_intakeRoller2StatorCurrent;
    private StatusSignal<Current> m_intakeRoller2SupplyCurrent;
    private StatusSignal<Voltage> m_intakeRoller2Voltage;

    private StatusSignal<AngularVelocity> m_intakePivotVelocity;
    private StatusSignal<Voltage> m_intakePivotVoltage;
    private StatusSignal<Current> m_intakePivotStatorCurrent;
    private StatusSignal<Current> m_intakePivotSupplyCurrent;
    private StatusSignal<Angle> m_intakePivotRotations;

    private double m_goalVelocity;
    private double m_goalPosition;
  
   private final int kIntakeRollerID = 56;
    private final int kIntakeRollerLeaderID = 57;
    private final int kIntakeExtenderID = 55;

    private final double m_gearBox = 1.0/25.0;

    private final double m_gearRatio = m_gearBox;

    private final String kCanbus = "rio";

    public IntakeIOTalonFX() {
        m_intakeRollerLeft = new TalonFX(kIntakeRollerLeaderID, kCanbus);
        m_intakeRoller = new TalonFX(kIntakeRollerID, kCanbus);
        m_intakeExtender = new TalonFX(kIntakeExtenderID, kCanbus);

        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 20.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.Slot0.kP = 10;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0;

        final TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        rollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerCfg.Slot0.kP = 10;
        rollerCfg.Slot0.kI = 0;
        rollerCfg.Slot0.kD = 0.1;

        final MotorOutputConfigs extenderConfigs = new MotorOutputConfigs();
        extenderConfigs.NeutralMode = NeutralModeValue.Brake;

        m_intakeExtender.getConfigurator().apply(extenderConfigs);
        m_intakeExtender.getConfigurator().apply(cfg);
        m_intakeRoller.getConfigurator().apply(rollerCfg);
        m_intakeRollerLeft.getConfigurator().apply(rollerCfg);
       

        m_intakeRoller.setControl(new Follower(kIntakeRollerLeaderID, MotorAlignmentValue.Opposed));

        m_extenderRequest = new PositionVoltage(0).withSlot(0);

        m_intakeRoller1Velocity = m_intakeRollerLeft.getVelocity();
        m_intakeRoller1StatorCurrent = m_intakeRollerLeft.getStatorCurrent();
        m_intakeRoller1SupplyCurrent = m_intakeRollerLeft.getSupplyCurrent();
        m_intakeRoller1Voltage = m_intakeRollerLeft.getSupplyVoltage();

        m_intakeRoller2Velocity = m_intakeRoller.getVelocity();
        m_intakeRoller2StatorCurrent = m_intakeRoller.getStatorCurrent();
        m_intakeRoller2SupplyCurrent = m_intakeRoller.getSupplyCurrent();
        m_intakeRoller2Voltage = m_intakeRoller.getSupplyVoltage();

        m_intakePivotVelocity = m_intakeExtender.getVelocity();
        m_intakePivotVoltage = m_intakeExtender.getSupplyVoltage();
        m_intakePivotStatorCurrent = m_intakeExtender.getStatorCurrent();
        m_intakePivotSupplyCurrent = m_intakeExtender.getSupplyCurrent();
        m_intakePivotRotations = m_intakeExtender.getPosition();

        m_goalVelocity = 0;
        m_goalPosition = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
            m_intakeRoller1Velocity,
            m_intakeRoller1StatorCurrent,
            m_intakeRoller1SupplyCurrent,
            m_intakeRoller1Voltage,
            m_intakeRoller2Velocity,
            m_intakeRoller2StatorCurrent,
            m_intakeRoller2SupplyCurrent,
            m_intakeRoller2Voltage,
            m_intakePivotVelocity,
            m_intakePivotVoltage,
            m_intakePivotStatorCurrent,
            m_intakePivotSupplyCurrent,
            m_intakePivotRotations
        );

        m_intakeExtender.setPosition(0, 2);
    }

    @Override
    public void setSpeed(double speed) {
        m_goalVelocity = speed;
        m_intakeRollerLeft.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setExtenderSetpoint(double rotations) {
        m_goalPosition = rotations;
        m_intakeExtender.setControl(m_extenderRequest.withPosition(rotations));
    }

    public void setPosition(double position) {
        m_intakeExtender.setPosition(position);
    }

    // @Override
    // public void setVelocity(double velocity) {
    //     m_intakeRollerLeft.setControl(m_velocityTorque.withVelocity(Units.RotationsPerSecond.of(velocity)));
    // }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs) {
        m_inputs.intakeRoller1Connected = 
            BaseStatusSignal.refreshAll(
                m_intakeRoller1Velocity,
                m_intakeRoller1Voltage,
                m_intakeRoller1StatorCurrent,
                m_intakeRoller1SupplyCurrent
            ).isOK();
        
        m_inputs.intakeRoller1VelocityRPS = m_intakeRoller1Velocity.getValueAsDouble();
        m_inputs.intakeRoller1StatorCurrent = m_intakeRoller1StatorCurrent.getValueAsDouble();
        m_inputs.intakeRoller1SupplyCurrent = m_intakeRoller1SupplyCurrent.getValueAsDouble();
        m_inputs.intakeRoller1Voltage = m_intakeRoller1Voltage.getValueAsDouble();

        m_inputs.intakeRoller2Connected = 
            BaseStatusSignal.refreshAll(
                m_intakeRoller2Velocity,
                m_intakeRoller2Voltage,
                m_intakeRoller2StatorCurrent,
                m_intakeRoller2SupplyCurrent
            ).isOK();
        
        m_inputs.intakeRoller2VelocityRPS = m_intakeRoller2Velocity.getValueAsDouble();
        m_inputs.intakeRoller2StatorCurrent = m_intakeRoller2StatorCurrent.getValueAsDouble();
        m_inputs.intakeRoller2SupplyCurrent = m_intakeRoller2SupplyCurrent.getValueAsDouble();
        m_inputs.intakeRoller2Voltage = m_intakeRoller2Voltage.getValueAsDouble();

        m_inputs.intakePivotConnected = 
            BaseStatusSignal.refreshAll(
                m_intakePivotVelocity,
                m_intakePivotVoltage,
                m_intakePivotStatorCurrent,
                m_intakePivotSupplyCurrent,
                m_intakePivotRotations
            ).isOK();
        
        m_inputs.intakePivotVelocityRPS = m_intakePivotVelocity.getValueAsDouble();
        m_inputs.intakePivotStatorCurrent = m_intakePivotStatorCurrent.getValueAsDouble();
        m_inputs.intakePivotSupplyCurrent = m_intakePivotSupplyCurrent.getValueAsDouble();
        m_inputs.intakePivotVoltage = m_intakePivotVoltage.getValueAsDouble();
        m_inputs.intakePivotRotations = m_intakePivotRotations.getValueAsDouble();

        m_inputs.goalVelocity = m_goalVelocity;
        m_inputs.goalPosition = m_goalPosition;
    }
}