package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReasonValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX m_intakeRoller;
    private final TalonFX m_intakeExtender;
    private final TalonFX m_intakeRollerLeft;
    private PositionVoltage m_extenderRequest;
  
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
        cfg.CurrentLimits.SupplyCurrentLimit = 5.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.Slot0.kP = 0.3;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0;

        final MotorOutputConfigs extenderConfigs = new MotorOutputConfigs();
        extenderConfigs.NeutralMode = NeutralModeValue.Brake;

        final TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // final Slot0Configs extenderSlot0Configs = new Slot0Configs();
        // extenderSlot0Configs.kP = 10;


        m_intakeExtender.getConfigurator().apply(cfg);
       m_intakeExtender.getConfigurator().apply(extenderConfigs);

       m_intakeRoller.getConfigurator().apply(config);
       m_intakeRollerLeft.getConfigurator().apply(config);



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
    public void updateInputs(IntakeIOInputsAutoLogged m_inputs) {
        m_inputs.speed = m_intakeRoller.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_intakeRoller.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_intakeRoller.getSupplyCurrent().getValueAsDouble();
        m_inputs.position = m_intakeExtender.getPosition().getValueAsDouble();
    }
    
}
