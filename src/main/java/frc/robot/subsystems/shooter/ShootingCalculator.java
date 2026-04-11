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

    // private double inmin = 1000.0;
    // private double inmax = 2000.0;
    // private double outmin = 1000.0;
    // private double outmax = 1500.0;


       
    public ShootingCalculator(CommandSwerveDrivetrain drivetrain){
        //Hub Interp v4
        // shotHoodAngleMap.put(Units.inchesToMeters(63.625), 1100.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(103.625), 1300.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(103.625+40), 1650.0);
        // shotHoodAngleMap.put(Units.inchesToMeters(103.625+40+40), 1925.0);

        //Corner Interp V4
        shotHoodAngleMap.put(6.5, 2.2);
        shotHoodAngleMap.put(9.7, 2.2);
        // shotHoodAngleMap.put(8.15, 2000.0);
        // shotHoodAngleMap.put(9.25, 2000.0);
        // shotHoodAngleMap.put(10.9, 2000.0);
        
        m_drivetrain = drivetrain;


        //Hub Interp V4
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(63.625), 47.0);
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625), 52.0);
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40), 55.0);
        // shotFlywheelSpeedMap.put(Units.inchesToMeters(103.625+40+40), 60.0);

        //Corner Interp V4
        shotFlywheelSpeedMap.put(6.5, 32.0);
        shotFlywheelSpeedMap.put(9.7, 41.0);
        // shotFlywheelSpeedMap.put(8.15, 70.0);
        // shotFlywheelSpeedMap.put(9.25, 80.0);
        // shotFlywheelSpeedMap.put(10.9, 95.0);

    }
    
    // public double map(double x) {
    //     return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
    // }

    
    public double getHoodPosition(){
        return shotHoodAngleMap.get(m_drivetrain.getDistanceFromAllianceWall());
    }

    public double getShooterSpeed(){
        return shotFlywheelSpeedMap.get(m_drivetrain.getDistanceFromAllianceWall());
    }
}