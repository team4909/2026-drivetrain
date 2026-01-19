package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ShooterIOTalonFX implements ShooterIO {


    private final TalonFX m_shootermotor1;
    private final TalonFX m_shootermotor2;
    private final int kShooterMotor1ID = 27;
    private final int kShooterMotor2ID = 26;
    private final String kCanbus = "CANivore2";



    public ShooterIOTalonFX() {
        m_shootermotor1 = new TalonFX(kShooterMotor1ID, kCanbus);
        m_shootermotor2 = new TalonFX(kShooterMotor2ID, kCanbus);

        final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_shootermotor1.getConfigurator().apply(shooterMotorConfig);
        m_shootermotor2.getConfigurator().apply(shooterMotorConfig);

 
        m_shootermotor2.setControl(new Follower(kShooterMotor1ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setSpeed(double speed) {
     
        m_shootermotor1.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_shootermotor1.setNeutralMode(neutralModeValue);
        m_shootermotor2.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(ShooterIOInputsAutoLogged m_inputs) {
        m_inputs.speed = m_shootermotor1.getVelocity().getValueAsDouble();
        m_inputs.statorCurrent = m_shootermotor1.getStatorCurrent().getValueAsDouble();
        m_inputs.supplyCurrent = m_shootermotor1.getSupplyCurrent().getValueAsDouble();
    }
}
