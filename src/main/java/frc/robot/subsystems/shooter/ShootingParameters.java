// import java.util.Map;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
// import edu.wpi.first.math.util.Units;
// import frc.robot.subsystems.hood.Hood;
// import frc.robot.subsystems.turret.Turret;

// public class ShootingParameters {
//         private static final InterpolatingTreeMap<Double, ShooterParameters> shooterTable = new InterpolatingTreeMap<>();
//         static{
//             shooterTable.put(Units.inchesToMeters(63.625), new ShooterParameters(47.0, 25.0, 1.2));
//             shooterTable.put(Units.inchesToMeters(103.625), new ShooterParameters(52.0, 25.0, 1.2));
//             shooterTable.put(Units.inchesToMeters(103.625+40), new ShooterParameters(55.0, 25.0, 1.2));
//             shooterTable.put(Units.inchesToMeters(103.625+40+40), new ShooterParameters(60.0, 25.0, 1.2));
//         }

//         private Hood m_hood;
//         private Shooter m_shooter;

//         public ShootingParameters(Hood hood, Shooter shooter) {
//             m_hood = hood;
//             m_shooter = shooter;
//         }

//         public ShooterCommand calculate(
//             Translation2d robotPosition,
//             Translation2d robotVelocity,
//             Translation2d goalPosition,
//             double latencyCompensation
//         ) {
//             // 1. Project future position
//             Translation2d futurePos = robotPosition.plus(
//                 robotVelocity.times(latencyCompensation)
//             );

//             // 2. Get target vector
//             Translation2d toGoal = goalPosition.minus(futurePos);
//             double distance = toGoal.getNorm();
//             Translation2d targetDirection = toGoal.div(distance);

//             // 3. Look up baseline velocity from table   
//             ShooterParameters baseline = shooterTable.get(distance)
//             double ballReleaseAngle = 90 - baseline.hoodAngle(); // cos90 = 0, multiplying ball velocity by 0 means ball straight up.
//             double horizontalVelocity = distance / baseline.timeOfFlight() * Math.cos(Units.degreesToRadians(ballReleaseAngle)); //left off here 

//             // 4. Build target velocity vector
//             Translation2d targetVelocity = targetDirection.times(baselineVelocity);

//             // 5. THE MAGIC: subtract robot velocity
//             Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

//             // 6. Extract results
//             Rotation2d turretAngle = shotVelocity.getAngle();
//             double requiredVelocity = shotVelocity.getNorm();

//             // 7. Use table in reverse: velocity → effective distance → RPM
//             double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
//             double requiredRpm = shooterTable.get(effectiveDistance).rpm;

//             return new ShooterCommand(turretAngle, requiredRpm);
//         }

//          public double velocityToEffectiveDistance(double velocity) {
//         // Binary search or iterate through table to find distance
//         // where (distance / ToF) = velocity
//         // Most InterpolatingTreeMap implementations support inverse lookup
//         // or you can build a reverse map: velocity → distance

//         for (Map.Entry<Double, ShooterParams> entry : SHOOTER_MAP.entrySet()) {
//             double dist = entry.getKey();
//             double vel = dist / entry.getValue().timeOfFlight;
//             if (vel >= velocity) {
//                 return dist; // Interpolate for better accuracy
//             }
//         }
//         return SHOOTER_MAP.lastKey(); // Clamp to max
//     }
//     }

//     // Simple data class for the LUT
//     public record ShooterParameters(double rps, double hoodAngle, double timeOfFlight) {}
//     public record ShooterCommand(Rotation2d turretAngle, double rpm) {}