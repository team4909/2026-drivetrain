package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class TurretIOTalonFX implements TurretIO{
    private final TalonFX m_motor;
    private final double m_gearBox = 1.0/3;
    private final double m_smallGear = 10;
    private final double m_bigGear = 105;
    private final double m_gearRatio = (m_gearBox * (m_smallGear / m_bigGear));
    private PositionVoltage m_request = new PositionVoltage(0);

    private final int kShooterMotorID = 60;
    private final String kCanbus = "CANivore1";

    public TurretIOTalonFX (){
        m_motor = new TalonFX(kShooterMotorID, kCanbus);

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 10; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        // final TalonFXConfiguration turretConfigs = new TalonFXConfiguration();
        // turretConfigs.CurrentLimits.SupplyCurrentLimit = 10.0;
        // turretConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        // var feedbackConfigs = new FeedbackConfigs();
        // feedbackConfigs.SensorToMechanismRatio = m_gearRatio;


        // slot0Configs.MotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        final MotorOutputConfigs turretConfiguration = new MotorOutputConfigs();
        turretConfiguration.NeutralMode = NeutralModeValue.Brake;




        m_motor.getConfigurator().apply(slot0Configs);
        // m_motor.getConfigurator().apply(turretConfigs);
        m_motor.getConfigurator().apply(turretConfiguration);

        m_motor.setPosition(0, 2);
        // System.out.println("Starting position: "+ m_motor.getPosition());
    }

    public void setSpeed(double speed) {
        m_motor.setControl(new DutyCycleOut(speed));
    }

    public void setSetpoint(double rotations) {
        // System.out.println("rotations: " + rotations);
        // System.out.println("Gear Ratio: " + m_gearRatio);
        // System.out.println("Motor Pos: " + m_motor.getPosition().getValueAsDouble());
        // System.out.println("Goal: " + rotations/m_gearRatio);
        m_motor.setControl(m_request.withPosition(Rotations.of(rotations/m_gearRatio)));
    }

    public double getTurretPosition(){return m_motor.getPosition().getValueAsDouble()*m_gearRatio;}
}
