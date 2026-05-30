package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public boolean motor1Connected = false;
        public double motor1VelocityRPS = 0.0;
        public double motor1StatorCurrent = 0.0;
        public double motor1SupplyCurrent = 0.0;
        public double motor1Voltage = 0.0;

        public boolean motor2Connected = false;
        public double motor2VelocityRPS = 0.0;
        public double motor2StatorCurrent = 0.0;
        public double motor2SupplyCurrent = 0.0;
        public double motor2Voltage = 0.0;
        
        // public boolean motor3Connected = false;
        // public double motor3VelocityRPS = 0.0;
        // public double motor3StatorCurrent = 0.0;
        // public double motor3SupplyCurrent = 0.0;
        // public double motor3Voltage = 0.0;  

        public boolean motor4Connected = false;
        public double motor4VelocityRPS = 0.0;
        public double motor4StatorCurrent = 0.0;
        public double motor4SupplyCurrent = 0.0;
        public double motor4Voltage = 0.0;
        
        public boolean hoodRollerConnected = false;
        public double hoodRollerVelocityRPS = 0.0;
        public double hoodRollerStatorCurrent = 0.0;
        public double hoodRollerSupplyCurrent = 0.0;
        public double hoodRollerVoltage = 0.0;
        public double hoodRollerPIDOutput = 0.0;
        

        public double goalVelocity = 0.0;
    }

    // Methods are how we move actions to hardware
    public void setVelocity(double RPS);

    public void setBrakeMode(boolean enableBrakeMode);

    public void setDutyCycle(double dutyCycle);

    // Get Hardware Data to Business Logic
    public void updateInputs(ShooterIOInputsAutoLogged m_inputs);
}