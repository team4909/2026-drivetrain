package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;


@SuppressWarnings("static-access")
public class ShootingParameters {
    // private static final InterpolatingTreeMap<Double, ShooterParameters>
    // shooterTable = new InterpolatingTreeMap<Double, ShooterParameters>();\

    private static final InterpolatingDoubleTreeMap m_hoodTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shooterTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap m_timeOfFlightTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_horizontalVelocityToDistanceTable = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap m_leftRotationTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_rightRotationTable = new InterpolatingDoubleTreeMap();

    private LoggedNetworkNumber fudge = new LoggedNetworkNumber("/Tuning/kickerFudge", 0.03);

    private Turret m_turret;

    private final double kLATENCYCOMPENSATION = 0.1; //-1.2;//If shots land ahead, lower number. If shots land behind, increase number

    static {
        m_leftRotationTable.put(1.226, 0.0);
        m_leftRotationTable.put(2.149, 0.0);
        m_leftRotationTable.put(3.2, -3.0);
        m_leftRotationTable.put(3.82, -1.5);
        m_leftRotationTable.put(4.25, 0.0);
        m_leftRotationTable.put(4.71, 0.0);

        m_rightRotationTable.put(1.226, 0.0);
        m_rightRotationTable.put(2.149, 1.5);
        m_rightRotationTable.put(3.2, 3.0);
        m_rightRotationTable.put(3.82, 1.5);
        m_rightRotationTable.put(4.25, 4.0);
        m_rightRotationTable.put(4.71, 4.0);

        // m_hoodTable.put(Units.inchesToMeters(63.234), 1000.0);
        // m_hoodTable.put(Units.inchesToMeters(93.234), 1100.0);
        // m_hoodTable.put(Units.inchesToMeters(123.234), 1150.0);
        // m_hoodTable.put(Units.inchesToMeters(153.234), 1500.0);
        // m_hoodTable.put(Units.inchesToMeters(183.234), 1750.0);

        m_hoodTable.put(1.226, 0.0);
        m_hoodTable.put(2.149, 0.1);
        m_hoodTable.put(3.2, 0.3);
        m_hoodTable.put(3.82, 0.45);
        m_hoodTable.put(4.25, 0.55);
        m_hoodTable.put(4.71, 0.5);
        


        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25), 0.01);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20), 0.1);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20), 0.1);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20), 0.15);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20), 0.15);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20), 0.17);

        // // m_hoodTable.put(4.73, 0.3);
        // // m_hoodTable.put(4.92, 0.45);

        
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20), 0.4);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 0.52);
        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 0.54);

        // m_hoodTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 0.5);



        // m_hoodTable.put(Units.inchesToMeters(103.625+40), 1650.0);
        // // m_hoodTable.put(Units.inchesToMeters(103.625+40+40), 1925.0);
        // m_hoodTable.put(Units.inchesToMeters(183.234), 1750.0);

        // m_shooterTable.put(Units.inchesToMeters(63.234), 47.0);
        // m_shooterTable.put(Units.inchesToMeters(93.234), 49.0);
        // m_shooterTable.put(Units.inchesToMeters(123.234), 54.0);
        // m_shooterTable.put(Units.inchesToMeters(153.234), 57.0);
        // m_shooterTable.put(Units.inchesToMeters(183.234), 65.0);

        m_shooterTable.put(1.226, 40.0);
        m_shooterTable.put(2.149,45.0);
        m_shooterTable.put(3.2, 48.0);
        m_shooterTable.put(3.82, 52.0);
        m_shooterTable.put(4.25, 55.0);
        m_shooterTable.put(4.71, 63.0);


        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25), 42.0);//*1.05);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20), 45.0);//*1.05);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20), 49.0);//*1.05);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20), 52.0);//*1.05);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20), 55.0);//*1.05);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20), 60.0);//*1.05);

        // // m_shooterTable.put(4.73, 60.0);
        // // m_shooterTable.put(4.92, 60.0);
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20), 62.0); //64.5*1.17
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20), 65.5); //74.0*1.2
        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 80.0); //95

        // m_shooterTable.put(Units.inchesToMeters(32+26.75+13.25+20+20+20+20+20+20+20+20), 64.0);


        // m_shooterTable.put(Units.inchesToMeters(103.625+40), 53.0); //55
        // // m_shooterTable.put(Units.inchesToMeters(103.625+40+40), 60.0);
        // m_shooterTable.put(Units.inchesToMeters(183.234), 63.0; //65

        m_timeOfFlightTable.put(1.226, 0.98);
        m_timeOfFlightTable.put(2.149, 1.04);
        m_timeOfFlightTable.put(3.2, 1.08);
        m_timeOfFlightTable.put(3.82, 1.12);
        m_timeOfFlightTable.put(4.25, 1.17);
        m_timeOfFlightTable.put(4.71, 1.32);


        // m_timeOfFlightTable.put(2.5, 0.98);
        // m_timeOfFlightTable.put(3.73, 1.2);
        // m_timeOfFlightTable.put(4.53, 1.34);
        // m_timeOfFlightTable.put(5.36, 1.25);//1.16);
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

    public ShootingParameters(CommandSwerveDrivetrain drivetrain, Turret turret) {
        m_drivetrain = drivetrain;
        m_turret = turret;
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

    // Logger.recordOutput("/shootingParams/applyingFudge", false);

    if(m_turret.getTurretPosition() > 0){
        requiredRPS = requiredRPS + m_rightRotationTable.get(effectiveDistance) * Math.sin(Math.toRadians(m_turret.getTurretPosition()*360.0));
        // Logger.recordOutput("/shootingParams/fudgeAmount", requiredRPS * fudge.getAsDouble() * Math.sin(Math.toRadians(m_turret.getTurretPosition()/360.0)));
        // Logger.recordOutput("/shootingParams/applyingFudge", true);
    }
    else if(m_turret.getTurretPosition() < 0){
        requiredRPS = requiredRPS + m_leftRotationTable.get(effectiveDistance) * Math.sin(Math.toRadians(m_turret.getTurretPosition()*360.0));
        // Logger.recordOutput("/shootingParams/fudgeAmount", requiredRPS * fudge.getAsDouble() * Math.sin(Math.toRadians(m_turret.getTurretPosition()/360.0)));
        // Logger.recordOutput("/shootingParams/applyingFudge", true);
    }

    Logger.recordOutput("/shootingParams/reqRPS", requiredRPS);
    Logger.recordOutput("/shootingParams/turretPos", m_turret.getTurretPosition());

    
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