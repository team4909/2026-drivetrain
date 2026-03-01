package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

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

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class IntakeIOTalonFX implements IntakeIO {

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

        m_intakeRoller.getConfigurator().apply(rollerCfg);
        m_intakeRollerLeft.getConfigurator().apply(rollerCfg);
       m_intakeExtender.getConfigurator().apply(extenderConfigs);

        m_intakeRoller.setControl(new Follower(kIntakeRollerLeaderID, MotorAlignmentValue.Opposed));

        m_extenderRequest = new PositionVoltage(0).withSlot(0);
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
        m_inputs.speed = m_intakeRollerLeft.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_intakeRollerLeft.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_intakeRollerLeft.getSupplyCurrent().getValueAsDouble();
        m_inputs.position = m_intakeExtender.getPosition().getValueAsDouble();
        m_inputs.velocity = m_intakeExtender.getVelocity().getValueAsDouble();
    }
}