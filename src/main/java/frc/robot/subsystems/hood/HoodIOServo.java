package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO{
    private int servoLeftChannel = 1;
    private int servoRightChannel = 2;
    private Servo m_servoLeft;
    private Servo m_servoRight;

    public HoodIOServo(){
        m_servoLeft = new Servo(servoLeftChannel);
        m_servoRight = new Servo(servoRightChannel);

    }

    public void setPosition(double position){
        m_servoLeft.set(position);
        m_servoRight.set(position);

    }
public void updateInputs(HoodIOInputsAutoLogged m_inputs){

    m_inputs.positionLeftActuator = m_servoLeft.getPosition();
    m_inputs.positionRightActuator = m_servoRight.getPosition();
}

}