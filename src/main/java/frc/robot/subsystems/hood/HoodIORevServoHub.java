package frc.robot.subsystems.hood;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

public class HoodIORevServoHub implements HoodIO{
    private ServoChannel m_servoLeftChannel;
    private ServoChannel m_servoRightChannel;
    private ServoHubConfig m_config = new ServoHubConfig();
    private ServoHub m_servoHub;

    public HoodIORevServoHub(){
        // m_servoLeft = new Servo(servoLeftChannel);
        // m_servoRight = new Servo(servoRightChannel);
        m_config.channel0.pulseRange(500,1500,2500)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        m_servoHub = new ServoHub(1);

        m_servoHub.configure(m_config, ServoHub.ResetMode.kResetSafeParameters);

        m_servoLeftChannel = m_servoHub.getServoChannel(ChannelId.kChannelId0);
        m_servoRightChannel = m_servoHub.getServoChannel(ChannelId.kChannelId1);

        m_servoLeftChannel.setPowered(true);
        m_servoRightChannel.setPowered(true);

        m_servoLeftChannel.setEnabled(true);
        m_servoRightChannel.setEnabled(true);
    }

    public void setPosition(int position){
        //0-1 scale of extention
        // m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, (int)(500+(position*2000)));
        m_servoLeftChannel.setPulseWidth(position);
        // m_servoRightChannel.setPulseWidth((int)(500+(position*2000)));

    }
// public void updateInputs(HoodIOInputsAutoLogged m_inputs){

    // m_inputs.positionLeftActuator = m_servoLeft.getPosition();
    // m_inputs.positionRightActuator = m_servoRight.getPosition();
}