package frc.robot.subsystems.hood;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

public class HoodIORevServoHub implements HoodIO {
    private ServoChannel m_servoLeftChannel;
    private ServoChannel m_servoRightChannel;
    private ServoHubConfig m_config = new ServoHubConfig();
    private ServoHub m_servoHub;

    public HoodIORevServoHub() {
        // m_servoLeft = new Servo(servoLeftChannel);
        // m_servoRight = new Servo(servoRightChannel);
        m_config.channel3.pulseRange(1000, 1500, 2000)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        m_config.channel4.pulseRange(1000, 1500, 2000)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        m_servoHub = new ServoHub(1);

        m_servoHub.configure(m_config, ServoHub.ResetMode.kResetSafeParameters);

        m_servoLeftChannel = m_servoHub.getServoChannel(ChannelId.kChannelId3);
        m_servoRightChannel = m_servoHub.getServoChannel(ChannelId.kChannelId4);

        m_servoLeftChannel.setPowered(true);
        m_servoRightChannel.setPowered(true);

        m_servoLeftChannel.setEnabled(true);
        m_servoRightChannel.setEnabled(true);
    }

    public void setPosition(int position) {
        // 0-1 scale of extention
        // m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2,
        // (int)(500+(position*2000)));
        try {
            // pulse period may be making servo faster although unclear
            m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank3_5, 20000);

            m_servoLeftChannel.setPulseWidth(position);
            m_servoRightChannel.setPulseWidth(position);

        } catch (Exception e) {
            System.out.print("Error in set position: " + e);
        }
        // m_servoRightChannel.setPulseWidth((int)(500+(position*2000)));

    }

    // TODO: change value from pulse width to angle in degrees
    public void updateInputs(HoodIOInputsAutoLogged m_inputs) {
        m_inputs.hoodAngleDegrees = m_servoLeftChannel.getPulseWidth();
    }
}