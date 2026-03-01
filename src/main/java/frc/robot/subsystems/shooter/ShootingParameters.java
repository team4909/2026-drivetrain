package frc.robot.subsystems.shooter;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

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

    private final double kLATENCYCOMPENSATION = -1.2;//If shots land ahead, lower number. If shots land behind, increase number

    static {
        m_hoodTable.put(Units.inchesToMeters(63.234), 1000.0);
        m_hoodTable.put(Units.inchesToMeters(93.234), 1100.0);
        m_hoodTable.put(Units.inchesToMeters(123.234), 1150.0);
        m_hoodTable.put(Units.inchesToMeters(153.234), 1500.0);
        m_hoodTable.put(Units.inchesToMeters(183.234), 1750.0);

        m_shooterTable.put(Units.inchesToMeters(63.234), 47.0);
        m_shooterTable.put(Units.inchesToMeters(93.234), 49.0);
        m_shooterTable.put(Units.inchesToMeters(123.234), 54.0);
        m_shooterTable.put(Units.inchesToMeters(153.234), 57.0);
        m_shooterTable.put(Units.inchesToMeters(183.234), 65.0);


        m_timeOfFlightTable.put(Units.inchesToMeters(63.234), 0.93);
        m_timeOfFlightTable.put(Units.inchesToMeters(93.234), 1.11);
        m_timeOfFlightTable.put(Units.inchesToMeters(123.234), 1.22);
        m_timeOfFlightTable.put(Units.inchesToMeters(153.234), 1.22);
        m_timeOfFlightTable.put(Units.inchesToMeters(183.234), 1.24);


        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(63.234)/0.93, Units.inchesToMeters(63.234));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(93.234)/1.11, Units.inchesToMeters(93.234));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(123.234)/1.22, Units.inchesToMeters(123.234));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(153.234)/1.22, Units.inchesToMeters(153.234));
        m_horizontalVelocityToDistanceTable.put(Units.inchesToMeters(183.234)/1.24, Units.inchesToMeters(183.234));
    }

    private CommandSwerveDrivetrain m_drivetrain;

    public ShootingParameters(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public ShooterCommand calculate(Translation2d goalPosition) {

        SwerveDriveState drivetrainState = m_drivetrain.getState();

        Translation2d robotPosition = drivetrainState.Pose.getTranslation();
        Logger.recordOutput("ShootOnTheMove/currentPose", robotPosition);

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

        Logger.recordOutput("ShootOnTheMove/hoodAngle", requiredHoodAngle);
        Logger.recordOutput("ShootOnTheMove/requiredRPS", requiredRPS);
        Logger.recordOutput("ShootOnTheMove/effectiveDistance", effectiveDistance);
        Logger.recordOutput("ShootOnTheMove/hubDistance", distance);
        Logger.recordOutput("ShootOnTheMove/futurePose", futurePos);
        Logger.recordOutput("ShootOnTheMove/turretAngle", turretAngle.getDegrees());


        return new ShooterCommand(turretAngle, requiredRPS, requiredHoodAngle);
    }

    // Simple data class for the LUT
    public record ShooterParameters(double rps, double hoodAngle, double timeOfFlight) {
    }

    //Returned requirements
    public record ShooterCommand(Rotation2d turretAngle, double rpm, double hoodAngle) {
    }
}