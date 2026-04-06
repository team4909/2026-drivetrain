package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ShooterIOTalonFX implements ShooterIO {


    private final TalonFX m_shootermotor1;
    private final TalonFX m_shootermotor2;
    private final TalonFX m_shootermotor3;
    private final TalonFX m_shootermotor4;
    private final TalonFX m_hoodRoller;
    private final int kShooterMotor1ID = 27;
    private final int kShooterMotor2ID = 26;
    private final int kShooterMotor3ID = 99;
    private final int kShooterMotor4ID = 98;
    private final int kHoodRollerID = 87;
    private final String kCanbus = "CANivore2";

    private StatusSignal<AngularVelocity> m_shooter1Velocity;
    private StatusSignal<Current> m_shooter1StatorCurrent;
    private StatusSignal<Current> m_shooter1SupplyCurrent;
    private StatusSignal<Voltage> m_shooter1Voltage;

    private StatusSignal<AngularVelocity> m_shooter2Velocity;
    private StatusSignal<Current> m_shooter2StatorCurrent;
    private StatusSignal<Current> m_shooter2SupplyCurrent;
    private StatusSignal<Voltage> m_shooter2Voltage;

    private StatusSignal<AngularVelocity> m_shooter3Velocity;
    private StatusSignal<Current> m_shooter3StatorCurrent;
    private StatusSignal<Current> m_shooter3SupplyCurrent;
    private StatusSignal<Voltage> m_shooter3Voltage;

    private StatusSignal<AngularVelocity> m_shooter4Velocity;
    private StatusSignal<Current> m_shooter4StatorCurrent;
    private StatusSignal<Current> m_shooter4SupplyCurrent;
    private StatusSignal<Voltage> m_shooter4Voltage;

    private StatusSignal<AngularVelocity> m_hoodRollerVelocity;
    private StatusSignal<Current> m_hoodRollerStatorCurrent;
    private StatusSignal<Current> m_hoodRollerSupplyCurrent;
    private StatusSignal<Voltage> m_hoodRollerVoltage;


    private double m_goalVelocity;

    //Using Torque control because less Feedforwards, more efficient, and easier to tune
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0.0).withSlot(0);


    public ShooterIOTalonFX() {
        m_shootermotor1 = new TalonFX(kShooterMotor1ID, kCanbus);
        m_shootermotor2 = new TalonFX(kShooterMotor2ID, kCanbus);
        m_shootermotor3 = new TalonFX(kShooterMotor3ID, kCanbus);
        m_shootermotor4 = new TalonFX(kShooterMotor4ID, kCanbus);
        m_hoodRoller = new TalonFX(kHoodRollerID, kCanbus);
    

        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

        //All Feedforwards are in AMPS bc this is torque control

        shooterConfigs.Slot0.kP = 10.0;
        shooterConfigs.Slot0.kI = 0.0; 
        shooterConfigs.Slot0.kD = 0.0;
        shooterConfigs.Slot0.kS = 5.0;
        shooterConfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Units.Amps.of(40)).withPeakReverseTorqueCurrent(Units.Amps.of(-40));
        m_shootermotor1.getConfigurator().apply(shooterConfigs);
        m_shootermotor2.getConfigurator().apply(shooterConfigs);
        m_shootermotor3.getConfigurator().apply(shooterConfigs);
        m_shootermotor4.getConfigurator().apply(shooterConfigs);
        m_hoodRoller.getConfigurator().apply(shooterConfigs);

        // m_shootermotor2.setControl(new Follower(kShooterMotor1ID, MotorAlignmentValue.Opposed));
        // m_shootermotor3.setControl(new Follower(kShooterMotor1ID, MotorAlignmentValue.Opposed));
        // m_shootermotor4.setControl(new Follower(kShooterMotor1ID, MotorAlignmentValue.Aligned));
        // m_hoodRoller.setControl(new Follower(kShooterMotor1ID, MotorAlignmentValue.Aligned));

        m_shooter1Velocity = m_shootermotor1.getVelocity();
        m_shooter1StatorCurrent = m_shootermotor1.getStatorCurrent();
        m_shooter1SupplyCurrent = m_shootermotor1.getSupplyCurrent();
        m_shooter1Voltage = m_shootermotor1.getSupplyVoltage();

        m_shooter2Velocity = m_shootermotor2.getVelocity();
        m_shooter2StatorCurrent = m_shootermotor2.getStatorCurrent();
        m_shooter2SupplyCurrent = m_shootermotor2.getSupplyCurrent();
        m_shooter2Voltage = m_shootermotor2.getSupplyVoltage();

        m_shooter3Velocity = m_shootermotor3.getVelocity();
        m_shooter3StatorCurrent = m_shootermotor3.getStatorCurrent();
        m_shooter3SupplyCurrent = m_shootermotor3.getSupplyCurrent();
        m_shooter3Voltage = m_shootermotor3.getSupplyVoltage();

        m_shooter4Velocity = m_shootermotor4.getVelocity();
        m_shooter4StatorCurrent = m_shootermotor4.getStatorCurrent();
        m_shooter4SupplyCurrent = m_shootermotor4.getSupplyCurrent();
        m_shooter4Voltage = m_shootermotor4.getSupplyVoltage();

        m_hoodRollerVelocity = m_hoodRoller.getVelocity();
        m_hoodRollerStatorCurrent = m_hoodRoller.getStatorCurrent();
        m_hoodRollerSupplyCurrent = m_hoodRoller.getSupplyCurrent();
        m_hoodRollerVoltage = m_hoodRoller.getSupplyVoltage();

        m_goalVelocity = 0;

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
            m_shooter1Velocity,
            m_shooter1StatorCurrent,
            m_shooter1SupplyCurrent,
            m_shooter1Voltage,
            m_shooter2Velocity,
            m_shooter2StatorCurrent,
            m_shooter2SupplyCurrent,
            m_shooter2Voltage,
            m_shooter3Velocity,
            m_shooter3StatorCurrent,
            m_shooter3SupplyCurrent,
            m_shooter3Voltage,
            m_shooter4Velocity,
            m_shooter4StatorCurrent,
            m_shooter4SupplyCurrent,
            m_shooter4Voltage,
            m_hoodRollerVelocity,
            m_hoodRollerStatorCurrent,
            m_hoodRollerSupplyCurrent,
            m_hoodRollerVoltage

        );
    }
    @Override
    public void setVelocity(double VelocityRPS) {
        m_goalVelocity = VelocityRPS;
        m_shootermotor1.setControl(m_velocityTorque.withVelocity(Units.RotationsPerSecond.of(VelocityRPS)));
    }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_shootermotor1.setNeutralMode(neutralModeValue);
        m_shootermotor2.setNeutralMode(neutralModeValue);
        m_shootermotor3.setNeutralMode(neutralModeValue);
        m_shootermotor4.setNeutralMode(neutralModeValue);
        m_hoodRoller.setNeutralMode(neutralModeValue);
    }

    public void updateInputs(ShooterIOInputsAutoLogged m_inputs) {
        m_inputs.motor1Connected = 
            BaseStatusSignal.refreshAll(    
                m_shooter1Velocity,
                m_shooter1Voltage,
                m_shooter1StatorCurrent,
                m_shooter1SupplyCurrent
            ).isOK();

        m_inputs.motor1VelocityRPS = m_shooter1Velocity.getValueAsDouble();
        m_inputs.motor1StatorCurrent = m_shooter1StatorCurrent.getValueAsDouble();
        m_inputs.motor1SupplyCurrent = m_shooter1SupplyCurrent.getValueAsDouble();
        m_inputs.motor1Voltage = m_shooter1Voltage.getValueAsDouble();

        m_inputs.motor2Connected = 
            BaseStatusSignal.refreshAll(
                m_shooter2Velocity,
                m_shooter2Voltage,
                m_shooter2StatorCurrent,
                m_shooter2SupplyCurrent
            ).isOK();
        m_inputs.motor2VelocityRPS = m_shooter2Velocity.getValueAsDouble();
        m_inputs.motor2StatorCurrent = m_shooter2StatorCurrent.getValueAsDouble();
        m_inputs.motor2SupplyCurrent = m_shooter2SupplyCurrent.getValueAsDouble();
        m_inputs.motor2Voltage = m_shooter2Voltage.getValueAsDouble();

        m_inputs.motor3Connected = 
            BaseStatusSignal.refreshAll(
                m_shooter3Velocity,
                m_shooter3Voltage,
                m_shooter3StatorCurrent,
                m_shooter3SupplyCurrent
            ).isOK();
        m_inputs.motor3VelocityRPS = m_shooter3Velocity.getValueAsDouble();
        m_inputs.motor3StatorCurrent = m_shooter3StatorCurrent.getValueAsDouble();
        m_inputs.motor3SupplyCurrent = m_shooter3SupplyCurrent.getValueAsDouble();
        m_inputs.motor3Voltage = m_shooter3Voltage.getValueAsDouble();

        m_inputs.motor4Connected = 
            BaseStatusSignal.refreshAll(
                m_shooter4Velocity,
                m_shooter4Voltage,
                m_shooter4StatorCurrent,
                m_shooter4SupplyCurrent
            ).isOK();
        m_inputs.motor4VelocityRPS = m_shooter4Velocity.getValueAsDouble();
        m_inputs.motor4StatorCurrent = m_shooter4StatorCurrent.getValueAsDouble();
        m_inputs.motor4SupplyCurrent = m_shooter4SupplyCurrent.getValueAsDouble();
        m_inputs.motor4Voltage = m_shooter4Voltage.getValueAsDouble();

        m_inputs.hoodRollerConnected = 
            BaseStatusSignal.refreshAll(
                m_hoodRollerVelocity,
                m_hoodRollerVoltage,
                m_hoodRollerStatorCurrent,
                m_hoodRollerSupplyCurrent
            ).isOK();
        m_inputs.hoodRollerVelocityRPS = m_hoodRollerVelocity.getValueAsDouble();
        m_inputs.hoodRollerStatorCurrent = m_hoodRollerStatorCurrent.getValueAsDouble();
        m_inputs.hoodRollerSupplyCurrent = m_hoodRollerSupplyCurrent.getValueAsDouble();
        m_inputs.hoodRollerVoltage = m_hoodRollerVoltage.getValueAsDouble();

        m_inputs.goalVelocity = m_goalVelocity;
    }
}
