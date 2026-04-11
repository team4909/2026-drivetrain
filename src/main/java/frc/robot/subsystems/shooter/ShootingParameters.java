package frc.robot.subsystems.shooter;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.geometry.Pose2d;
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
        m_hoodTable.put(1.5, 0.0);
        m_hoodTable.put(2.0, 0.25);
        m_hoodTable.put(2.5, 0.4);
        m_hoodTable.put(3.0, 0.6);
        m_hoodTable.put(3.5, 0.8);
        m_hoodTable.put(4.0, 1.25);
        m_hoodTable.put(4.5, 1.45);
        m_hoodTable.put(5.0, 1.7);
        m_hoodTable.put(5.65, 1.85);

        m_shooterTable.put(1.5, 27.5);
        m_shooterTable.put(2.0, 27.5);
        m_shooterTable.put(2.5, 30.5);
        m_shooterTable.put(3.0, 31.0);
        m_shooterTable.put(3.5, 33.0);
        m_shooterTable.put(4.0, 33.0);
        m_shooterTable.put(4.5, 34.0);
        m_shooterTable.put(5.0, 35.0);
        m_shooterTable.put(5.65, 38.0);

        m_timeOfFlightTable.put(2.5, 0.98);
        m_timeOfFlightTable.put(3.73, 1.2);
        m_timeOfFlightTable.put(4.53, 1.34);
        m_timeOfFlightTable.put(5.36, 1.25);//1.16);
        // m_timeOfFlightTable.put(Units.inchesToMeters(63.234), 0.93);
        // m_timeOfFlightTable.put(Units.inchesToMeters(93.234), 1.11);
        // m_timeOfFlightTable.put(Units.inchesToMeters(123.234), 1.22);
        // m_timeOfFlightTable.put(Units.inchesToMeters(153.234), 1.22-0.22);
        // m_timeOfFlightTable.put(Units.inchesToMeters(183.234), 1.24);


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
    Pose2d robotPose = drivetrainState.Pose;

    Translation2d robotPosition = drivetrainState.Pose.getTranslation();

    Translation2d robotVelocity = new Translation2d(
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vxMetersPerSecond, 
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, drivetrainState.Pose.getRotation()).vyMetersPerSecond
    );

    Rotation2d rotationalVelocity = new Rotation2d(
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrainState.Speeds, robotPose.getRotation()).omegaRadiansPerSecond
    );

    double timeOfFlight = m_timeOfFlightTable.get(robotPosition.getDistance(goalPosition));
    Translation2d futurePos = robotPosition;
    Rotation2d futureRotation = robotPose.getRotation();

    for (int i = 0; i < 3; i++) {
        futurePos = robotPosition.plus(robotVelocity.times(timeOfFlight + kLATENCYCOMPENSATION));
        futureRotation = robotPose.getRotation().plus(rotationalVelocity.times(timeOfFlight + kLATENCYCOMPENSATION));
        timeOfFlight = m_timeOfFlightTable.get(futurePos.getDistance(goalPosition));
    }

    // futurePos = new Pose2d(futurePos, );

    Translation2d toGoal = goalPosition.minus(futurePos);
    double effectiveDistance = toGoal.getNorm();

    Rotation2d turretAngle = toGoal.getAngle();//.minus(futureRotation); //TODO: see if I need to add or subtract future rotation


    double requiredRPS = m_shooterTable.get(effectiveDistance);
    double requiredHoodAngle = m_hoodTable.get(effectiveDistance);

    // Logging
    Logger.recordOutput("ShootOnTheMove/futurePose", futurePos);
    Logger.recordOutput("ShootOnTheMove/turretAngleDeg", turretAngle.getDegrees());

    return new ShooterCommand(turretAngle, requiredRPS, requiredHoodAngle);
}

    // Simple data class for the LUT
    public record ShooterParameters(double rps, double hoodAngle, double timeOfFlight) {
    }

    //Returned requirements
    public record ShooterCommand(Rotation2d turretAngle, double rpm, double hoodAngle) {
    }
}