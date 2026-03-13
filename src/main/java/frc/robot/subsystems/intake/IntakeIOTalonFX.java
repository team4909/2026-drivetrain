package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

    private StatusSignal<AngularVelocity> m_intakeRollerRightVelocity;
    private StatusSignal<Current> m_intakeRollerRightStatorCurrent;
    private StatusSignal<Voltage> m_intakeRollerRightVoltage;

    private StatusSignal<AngularVelocity> m_intakeRollerLeftVelocity;
    private StatusSignal<Current> m_intakeRollerLeftStatorCurrent;
    private StatusSignal<Voltage> m_intakeRollerLeftVoltage;

    private StatusSignal<AngularVelocity> m_intakeExtenderVelocity;
    private StatusSignal<Current> m_intakeExtenderStatorCurrent;
    private StatusSignal<Angle> m_intakeExtenderPosition;
    private StatusSignal<Voltage> m_intakeExtenderVoltage;

  private final TalonFX m_intakeRoller;
    private final TalonFX m_intakeExtender;
    private final TalonFX m_intakeRollerLeft;
    private PositionVoltage m_extenderRequest;
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  
   private final int kIntakeRollerID = 56;
    private final int kIntakeRollerLeaderID = 57;
    private final int kIntakeExtenderID = 55;

    private final double m_gearBox = 1.0/25.0;

    private final double m_gearRatio = m_gearBox;

    private final String kCanbus = "CANivore2";

    public IntakeIOTalonFX() {
        m_intakeRollerLeft = new TalonFX(kIntakeRollerLeaderID, kCanbus);
        m_intakeRoller = new TalonFX(kIntakeRollerID, kCanbus);
        m_intakeExtender = new TalonFX(kIntakeExtenderID, kCanbus);

        final TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.Slot0.kP = 1;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0;

        final TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        rollerCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerCfg.Slot0.kP = 10;
        rollerCfg.Slot0.kI = 0;
        rollerCfg.Slot0.kD = 0;

        final MotorOutputConfigs extenderConfigs = new MotorOutputConfigs();
        extenderConfigs.NeutralMode = NeutralModeValue.Brake;

        m_intakeExtender.getConfigurator().apply(extenderConfigs);
        m_intakeExtender.getConfigurator().apply(cfg);
        m_intakeRoller.getConfigurator().apply(rollerCfg);
        m_intakeRollerLeft.getConfigurator().apply(rollerCfg);
       

        m_intakeRoller.setControl(new Follower(kIntakeRollerLeaderID, MotorAlignmentValue.Opposed));

        m_extenderRequest = new PositionVoltage(0).withSlot(0);

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
            m_intakeRollerRightVelocity,
            m_intakeRollerRightStatorCurrent,
            m_intakeRollerRightVoltage,
            m_intakeRollerLeftVelocity,
            m_intakeRollerLeftStatorCurrent,
            m_intakeRollerLeftVoltage,
            m_intakeExtenderVelocity,
            m_intakeExtenderStatorCurrent,
            m_intakeExtenderPosition,
            m_intakeExtenderVoltage
        );
    }

    @Override
    public void setSpeed(double speed) {
        m_intakeRollerLeft.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setExtenderSetpoint(double rotations) {
        m_intakeExtender.setControl(m_extenderRequest.withPosition(rotations));
    }

    public void setPosition(double position) {
        m_intakeExtender.setPosition(position);
    }

    @Override
    public void setVelocity(double velocity) {
        m_intakeRollerLeft.setControl(m_velocityTorque.withVelocity(Units.RotationsPerSecond.of(velocity)));
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs) {
        m_inputs.intakeRollerRightConnected = 
            BaseStatusSignal.refreshAll(
                m_intakeRollerRightVelocity,
                m_intakeRollerRightStatorCurrent,
                m_intakeRollerRightVoltage
            ).isOK();
        m_inputs.intakeRollerLeftConnected = 
            BaseStatusSignal.refreshAll(
                m_intakeRollerLeftVelocity,
                m_intakeRollerLeftStatorCurrent,
                m_intakeRollerLeftVoltage
            ).isOK();
        m_inputs.intakeExtenderConnected = 
            BaseStatusSignal.refreshAll(
                m_intakeExtenderVelocity,
                m_intakeExtenderStatorCurrent,
                m_intakeExtenderPosition,
                m_intakeExtenderVoltage
            ).isOK();

            m_inputs.intakeRollerRightVelocity = m_intakeRollerRightVelocity.getValueAsDouble();
            m_inputs.statorCurrentIntakeRollerRight = m_intakeRollerRightStatorCurrent.getValueAsDouble();
            m_inputs.intakeRollerRightVoltage = m_intakeRollerRightVoltage.getValueAsDouble();

            m_inputs.intakeRollerLeftVelocity = m_intakeRollerLeftVelocity.getValueAsDouble();
            m_inputs.statorCurrentIntakeRollerLeft = m_intakeRollerLeftStatorCurrent.getValueAsDouble();
            m_inputs.intakeRollerLeftVoltage = m_intakeRollerLeftVoltage.getValueAsDouble();

            m_inputs.intakeExtenderVelocity = m_intakeExtenderVelocity.getValueAsDouble();
            m_inputs.statorCurrentIntakeExtender = m_intakeExtenderStatorCurrent.getValueAsDouble();
            m_inputs.IntakeExtenderPosition = m_intakeExtenderPosition.getValueAsDouble();
            m_inputs.intakeExtenderVoltage = m_intakeExtenderVoltage.getValueAsDouble();
    }
}