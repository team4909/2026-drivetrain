package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO {
    private int servoLeftChannel = 0;
    private int servoRightChannel = 1;
    private Servo m_servoLeft;
    private Servo m_servoRight;

    public HoodIOServo() {
        m_servoLeft = new Servo(servoLeftChannel);
        m_servoRight = new Servo(servoRightChannel);
        m_servoLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_servoRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    public void setPosition(double position) {

        // m_servoLeft.set(position);
        // m_servoRight.set(position);

        double setPos = MathUtil.clamp(position, 0, 50);
        m_servoLeft.setSpeed((setPos / 50 * 2) - 1);
    }
    // public void updateInputs(HoodIOInputsAutoLogged m_inputs){

    // m_inputs.positionLeftActuator = m_servoLeft.getPosition();
    // m_inputs.positionRightActuator = m_servoRight.getPosition();
    // }

}