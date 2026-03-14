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

    private final double kLATENCYCOMPENSATION = 0.1; //-1.2;//If shots land ahead, lower number. If shots land behind, increase number

    static {
        // m_hoodTable.put(Units.inchesToMeters(63.234), 1000.0);
        // m_hoodTable.put(Units.inchesToMeters(93.234), 1100.0);
        // m_hoodTable.put(Units.inchesToMeters(123.234), 1150.0);
        // m_hoodTable.put(Units.inchesToMeters(153.234), 1500.0);
        // m_hoodTable.put(Units.inchesToMeters(183.234), 1750.0);

        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25), 0.01);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20), 0.1);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20), 0.1);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20), 0.15);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20), 0.15);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20), 0.17);
        
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20), 0.32);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 0.4);
        m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 0.54);

        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 0.5);



        // m_hoodTable.put(Units.inchesToMeters(103.625+40), 1650.0);
        // // m_hoodTable.put(Units.inchesToMeters(103.625+40+40), 1925.0);
        // m_hoodTable.put(Units.inchesToMeters(183.234), 1750.0);

        // m_shooterTable.put(Units.inchesToMeters(63.234), 47.0);
        // m_shooterTable.put(Units.inchesToMeters(93.234), 49.0);
        // m_shooterTable.put(Units.inchesToMeters(123.234), 54.0);
        // m_shooterTable.put(Units.inchesToMeters(153.234), 57.0);
        // m_shooterTable.put(Units.inchesToMeters(183.234), 65.0);

        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25), 42.0*1.05);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20), 45.0*1.05);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20), 49.0*1.05);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20), 52.0*1.05);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20), 55.0*1.05);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20), 60.0*1.05);


        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20), 64.5*1.17); //64.5*1.17
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 74.0*1.2);
        m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 95.0);

        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 64.0);


        // m_shooterTable.put(Units.inchesToMeters(103.625+40), 53.0); //55
        // // m_shooterTable.put(Units.inchesToMeters(103.625+40+40), 60.0);
        // m_shooterTable.put(Units.inchesToMeters(183.234), 63.0); //65


        m_timeOfFlightTable.put(Units.inchesToMeters(63.234), 0.93);
        m_timeOfFlightTable.put(Units.inchesToMeters(93.234), 1.11);
        m_timeOfFlightTable.put(Units.inchesToMeters(123.234), 1.22);
        m_timeOfFlightTable.put(Units.inchesToMeters(153.234), 1.22);
        m_timeOfFlightTable.put(Units.inchesToMeters(183.234), 1.24-0.15);


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

    Translation2d robotVelocity = new Translation2d(
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vxMetersPerSecond, 
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vyMetersPerSecond
    );

    double timeOfFlight = m_timeOfFlightTable.get(robotPosition.getDistance(goalPosition));
    Translation2d futurePos = robotPosition;

    for (int i = 0; i < 3; i++) {
        futurePos = robotPosition.plus(robotVelocity.times(timeOfFlight + kLATENCYCOMPENSATION));
        timeOfFlight = m_timeOfFlightTable.get(futurePos.getDistance(goalPosition));
    }

    Translation2d toGoal = goalPosition.minus(futurePos);
    double effectiveDistance = toGoal.getNorm();

    Rotation2d turretAngle = toGoal.getAngle();

    double requiredRPS = m_shooterTable.get(effectiveDistance);
    double requiredHoodAngle = m_hoodTable.get(effectiveDistance);

    // Logging
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