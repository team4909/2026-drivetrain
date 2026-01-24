package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShootingCalculator {
    private final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
     new InterpolatingDoubleTreeMap();


        // shotHoodAngleMap.put(1.34, 0.2);
        // shotHoodAngleMap.put(1.78,0.2);
        // shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
        // shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
        // shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
        // shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
        // shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
        // shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
        // shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
        // shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));
    public ShootingCalculator(){
        shotFlywheelSpeedMap.put(1.34, 210.0);
        shotFlywheelSpeedMap.put(1.78, 220.0);
        shotFlywheelSpeedMap.put(2.17, 220.0);
        shotFlywheelSpeedMap.put(2.81, 230.0);
        shotFlywheelSpeedMap.put(3.82, 250.0);
        shotFlywheelSpeedMap.put(4.09, 255.0);
        shotFlywheelSpeedMap.put(4.40, 260.0);
        shotFlywheelSpeedMap.put(4.77, 265.0);
        shotFlywheelSpeedMap.put(5.57, 275.0);
        shotFlywheelSpeedMap.put(5.60, 290.0);
    }
    

    public double getHoodAngle(double distanceFromHub){
        return shotHoodAngleMap.get(distanceFromHub);
    }

    public double getShooterSpeed(double distanceFromHub){
        return shotFlywheelSpeedMap.get(distanceFromHub);
    }
}
