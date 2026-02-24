package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

@SuppressWarnings("static-access")
public class ShootingParameters {
    private static final InterpolatingDoubleTreeMap m_hoodTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_shooterTable = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap m_timeOfFlightTable = new InterpolatingDoubleTreeMap();

    private final double kLATENCYCOMPENSATION = 0.1;

    static {
        m_hoodTable.put(Units.inchesToMeters(63.625), 25.0);
        m_hoodTable.put(Units.inchesToMeters(103.625), 25.0);
        m_hoodTable.put(Units.inchesToMeters(103.625 + 40), 25.0);
        m_hoodTable.put(Units.inchesToMeters(103.625 + 40 + 40), 25.0);

        m_shooterTable.put(Units.inchesToMeters(63.625), 47.0);
        m_shooterTable.put(Units.inchesToMeters(103.625), 52.0);
        m_shooterTable.put(Units.inchesToMeters(103.625 + 40), 55.0);
        m_shooterTable.put(Units.inchesToMeters(103.625 + 40 + 40), 60.0);

        m_timeOfFlightTable.put(Units.inchesToMeters(63.625), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625 + 40), 1.2);
        m_timeOfFlightTable.put(Units.inchesToMeters(103.625 + 40 + 40), 1.2);
    }

    private CommandSwerveDrivetrain m_drivetrain;

    public ShootingParameters(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public ShooterCommand calculate(Translation2d goalPosition) {
        SwerveDriveState drivetrainState = m_drivetrain.getState();
        Translation2d robotPosition = drivetrainState.Pose.getTranslation();
        
        //Determine relative velocity of the target from the robot
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrainState.Speeds, 
            drivetrainState.Pose.getRotation()
        );
        Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        //Initial guess and Recursion
        double currentDistance = robotPosition.getDistance(goalPosition);
        double timeOfFlight = m_timeOfFlightTable.get(currentDistance);
        
        Translation2d virtualTarget = goalPosition;
        double virtualDistance = currentDistance;

        //3-5 times usually converges on the shot
        for (int i = 0; i < 5; i++) {
            double totalDelay = kLATENCYCOMPENSATION + timeOfFlight;
            virtualTarget = goalPosition.minus(robotVelocity.times(totalDelay));
            
            // Get the new distance to this virtual target
            virtualDistance = robotPosition.getDistance(virtualTarget);
            
            // Check the table for the TOF of the ADJUSTED shot
            timeOfFlight = m_timeOfFlightTable.get(virtualDistance);
        }

        // Once converged, use the control variables found in the LUT for the virtual distance
        Rotation2d turretAngle = virtualTarget.minus(robotPosition).getAngle();
        double requiredRPS = m_shooterTable.get(virtualDistance);
        double requiredHoodAngle = m_hoodTable.get(virtualDistance);

        return new ShooterCommand(turretAngle, requiredRPS, requiredHoodAngle);
    }

    public record ShooterParameters(double rps, double hoodAngle, double timeOfFlight) {}
    public record ShooterCommand(Rotation2d turretAngle, double rpm, double hoodAngle) {}
}