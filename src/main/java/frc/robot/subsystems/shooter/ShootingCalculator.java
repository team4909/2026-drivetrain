package frc.robot.subsystems.shooter;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import edu.wpi.first.math.util.Units;

public class ShootingCalculator {
    private final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
     new InterpolatingDoubleTreeMap();


       
    public ShootingCalculator(){
        shotHoodAngleMap.put(Units.inchesToMeters(87), 1000.0);
        shotHoodAngleMap.put(Units.inchesToMeters(92.3), 1100.0);
        shotHoodAngleMap.put(Units.inchesToMeters(97.6), 1200.0);
        shotHoodAngleMap.put(Units.inchesToMeters(102.9), 1300.0);
        shotHoodAngleMap.put(Units.inchesToMeters(108.2), 1400.0);
        shotHoodAngleMap.put(Units.inchesToMeters(113.5), 1500.0);
        shotHoodAngleMap.put(Units.inchesToMeters(122.52), 1600.0);
        shotHoodAngleMap.put(Units.inchesToMeters(131.54), 1700.0);
        shotHoodAngleMap.put(Units.inchesToMeters(140.56), 1800.0);
         shotHoodAngleMap.put(Units.inchesToMeters(149.58), 1900.0);
        shotHoodAngleMap.put(Units.inchesToMeters(158.6), 2000.0);



        shotFlywheelSpeedMap.put(Units.inchesToMeters(87), -50.0);
        shotFlywheelSpeedMap.put(1.78, -220.0);
        shotFlywheelSpeedMap.put(2.17, -220.0);
        shotFlywheelSpeedMap.put(2.88, -230.0);
        // shotFlywheelSpeedMap.put(3.82, 250.0);
        // shotFlywheelSpeedMap.put(4.09, 255.0);
        // shotFlywheelSpeedMap.put(4.40, 260.0);
        // shotFlywheelSpeedMap.put(4.77, 265.0);
        // shotFlywheelSpeedMap.put(5.57, 275.0);
        // shotFlywheelSpeedMap.put(5.60, 290.0);
    }
    

    
    public double getHoodPosition(double distanceFromHub){
        return shotHoodAngleMap.get(distanceFromHub);
    }

    public double getShooterSpeed(double distanceFromHub){
        return shotFlywheelSpeedMap.get(distanceFromHub);
    }
}