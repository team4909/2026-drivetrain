package frc.robot.subsystems.shooter;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ShootingCalculator {
    private final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
     new InterpolatingDoubleTreeMap();
    private CommandSwerveDrivetrain m_drivetrain;

       
    public ShootingCalculator(CommandSwerveDrivetrain drivetrain){
        //V3
        shotHoodAngleMap.put(Units.inchesToMeters(63.625), 1100.0);
        shotHoodAngleMap.put(Units.inchesToMeters(103.625), 1300.0);
        shotHoodAngleMap.put(Units.inchesToMeters(103.625+40), 1650.0);
        shotHoodAngleMap.put(Units.inchesToMeters(103.625+40+40), 1925.0);
        
        //V2
        // shotHoodAngleMap.put(Units.inchesToMeters(103.625), 1400.0);



        // shotHoodAngleMap.put(Units.inchesToMeters(97.6), 1200.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(102.9), 1300.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(108.2), 1400.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(113.5), 1500.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(122.52), 1600.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(131.54), 1700.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(140.56), 1800.0);
        //  shotHoodAngleMap.put(Units.inchesToMeters(149.58), 1900.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(158.6), 2000.0);
        m_drivetrain = drivetrain;


        //V3
        shotFlywheelSpeedMap.put(Units.inchesToMeters(63.625), 47.0);
        shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625), 52.0);
        shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40), 55.0);
        shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40+40), 60.0);

        //V2
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40+40), -63.0);
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40), -59.0);
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625), -53.0);

        // shotFlywheelSpeedMap.put(2.17, -220.0);
        // shotFlywheelSpeedMap.put(2.88, -230.0);
        // shotFlywheelSpeedMap.put(3.82, 250.0);
        // shotFlywheelSpeedMap.put(4.09, 255.0);
        // shotFlywheelSpeedMap.put(4.40, 260.0);
        // shotFlywheelSpeedMap.put(4.77, 265.0);
        // shotFlywheelSpeedMap.put(5.57, 275.0);
        // shotFlywheelSpeedMap.put(5.60, 290.0);
    }
    

    
    public double getHoodPosition(){
        return shotHoodAngleMap.get(m_drivetrain.getDistanceFromHub());
    }

    public double getShooterSpeed(){
        return shotFlywheelSpeedMap.get(m_drivetrain.getDistanceFromHub());
    }
}