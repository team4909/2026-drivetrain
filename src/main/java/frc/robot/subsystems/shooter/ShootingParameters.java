package frc.robot.subsystems.shooter;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;


@SuppressWarnings("static-access")
public class ShootingParameters {
    // private static final InterpolatingTreeMap<Double, ShooterParameters>
    // shooterTable = new InterpolatingTreeMap<Double, ShooterParameters>();\

    private static final InterpolatingDoubleTreeMap m_hoodTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shooterTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap m_timeOfFlightTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_horizontalVelocityToDistanceTable = new InterpolatingDoubleTreeMap();

    private final double kLATENCYCOMPENSATION = 0.1;//If shots land ahead, lower number. If shots land behind, increase number

    static {
        m_hoodTable.put(Units.inchesToMeters(63.625), 1100.0);
        m_hoodTable.put(Units.inchesToMeters(103.625), 1300.0);
        m_hoodTable.put(Units.inchesToMeters(103.625 + 40), 1650.0);
        m_hoodTable.put(Units.inchesToMeters(103.625 + 40 + 40), 1925.0);

        m_shooterTable.put(Units.inchesToMeters(63.625), 47.0);
        m_shooterTable.put(Units.inchesToMeters(103.625), 52.0);
        m_shooterTable.put(Units.inchesToMeters(103.625 + 40), 55.0);
        m_shooterTable.put(Units.inchesToMeters(103.625 + 40 + 40), 60.0);

        m_timeOfFlightTable.put(Units.inchesToMeters(63.625), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625 + 40), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625 + 40 + 40), 1.2);

        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(63.625)/1.2, Units.inchesToMeters(63.625));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(103.625)/1.2, Units.inchesToMeters(103.625));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(103.625 + 40)/1.2, Units.inchesToMeters(103.625 + 40));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(103.625 + 40 + 40)/1.2, Units.inchesToMeters(103.625 + 40 + 40));
    }

    private CommandSwerveDrivetrain m_drivetrain;

    public ShootingParameters(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public ShooterCommand calculate(Translation2d goalPosition) {

        SwerveDriveState drivetrainState = m_drivetrain.getState();

        Translation2d robotPosition = drivetrainState.Pose.getTranslation();
        Translation2d robotVelocity = new Translation2d(ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vxMetersPerSecond, ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vyMetersPerSecond);

        // --- ADDED RECURSION LOOP ---
        double predictedDistance = robotPosition.getDistance(goalPosition);
        double timeOfFlight = m_timeOfFlightTable.get(predictedDistance);
        Translation2d futurePos = robotPosition;

        for (int i = 0; i < 3; i++) {
            futurePos = robotPosition.plus(robotVelocity.times(kLATENCYCOMPENSATION + timeOfFlight));
            predictedDistance = futurePos.getDistance(goalPosition);
            timeOfFlight = m_timeOfFlightTable.get(predictedDistance);
        }
        // --- END RECURSION ---

        // 2. Get target vector
        Translation2d toGoal = goalPosition.minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        ShooterParameters baseline = new ShooterParameters(
                m_shooterTable.get(distance),
                m_hoodTable.get(distance),
                m_timeOfFlightTable.get(distance));

        double horizontalVelocity = distance / baseline.timeOfFlight();

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(horizontalVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        double effectiveDistance = m_horizontalVelocityToDistanceTable.get(requiredVelocity);
        double requiredRPS = m_shooterTable.get(effectiveDistance);
        double requiredHoodAngle = m_hoodTable.get(effectiveDistance);

        return new ShooterCommand(turretAngle, requiredRPS, requiredHoodAngle);
    }

    // Simple data class for the LUT
    public record ShooterParameters(double rps, double hoodAngle, double timeOfFlight) {
    }

    //Returned requirements
    public record ShooterCommand(Rotation2d turretAngle, double rpm, double hoodAngle) {
    }
}