package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOTalonFX implements TurretIO{
    private final TalonFX m_motor;
    private final double m_gearBox = 1.0/3;
    private final double m_smallGear = 10;
    private final double m_bigGear = 105;
    private final double m_gearRatio = m_gearBox * (m_smallGear / m_bigGear);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public TurretIOTalonFX (){
        m_motor = new TalonFX(1,"CANivore2");

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1d; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = m_gearRatio;

        m_motor.getConfigurator().apply(slot0Configs);
        m_motor.getConfigurator().apply(feedbackConfigs);
    }

    public void setSpeed(double speed) {
        m_motor.setControl(new DutyCycleOut(speed));
    }

    public void setSetpoint(double rotations) {
        m_motor.setControl(m_request.withPosition(rotations));
    }
}
