package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOTalonFX {
    private final TalonFX m_motor;

    public TurretIOTalonFX (){
        m_motor = new TalonFX(1,"CANivore2");

    }

    public void setSpeed(double speed) {
        m_motor.setControl(new DutyCycleOut(speed));
    }
}
