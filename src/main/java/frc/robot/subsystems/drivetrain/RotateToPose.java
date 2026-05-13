package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToPose extends Command {
  private CommandSwerveDrivetrain m_drivetrain;
  private Pose2d m_goalPose;
  private final ProfiledPIDController m_rotationController;
  private SwerveRequest.ApplyRobotSpeeds m_driveRequest;
  private Rotation2d m_targetHeading;

  private DoubleSupplier m_xVelocitySupplier;
  private DoubleSupplier m_yVelocitySupplier;

  private static InterpolatingDoubleTreeMap m_timeOfFlightTable = new InterpolatingDoubleTreeMap();

  public RotateToPose(CommandSwerveDrivetrain drivetrain, DoubleSupplier xVelocity, DoubleSupplier yVelocity) {
    m_drivetrain = drivetrain;
    m_xVelocitySupplier = xVelocity;
    m_yVelocitySupplier = yVelocity;

    m_goalPose = new Pose2d(drivetrain.getHubCenter(), new Rotation2d());
    m_rotationController = new ProfiledPIDController(10.0, 0.0, 0.01, new TrapezoidProfile.Constraints(100 * Math.PI / 10 * 2, 100 * Math.PI / 10 * 3));
    m_driveRequest = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    m_timeOfFlightTable.put(2.5, 0.98);
    m_timeOfFlightTable.put(3.73, 1.2);
    m_timeOfFlightTable.put(4.53, 1.34);
    m_timeOfFlightTable.put(5.36, 1.25);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d initialPose = m_drivetrain.getState().Pose;
    Pose2d goalPose = new Pose2d(m_drivetrain.getHubCenter(), new Rotation2d());

    Translation2d robotToTargetTranslation = poseInverse(new Pose2d(initialPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(goalPose.getTranslation(), new Rotation2d())).getTranslation();
    m_targetHeading = robotToTargetTranslation.getAngle();

    m_rotationController.setTolerance(Units.degreesToRadians(1.0));
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    ChassisSpeeds robotVelocity = getFieldRelativeChassisSpeeds(m_drivetrain.getState().Speeds, initialPose);

    m_rotationController.reset(initialPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getState().Pose;

    Pose2d goalPose = new Pose2d(m_drivetrain.getHubCenter(), new Rotation2d());

    Translation2d robotToTargetTranslation = poseInverse(new Pose2d(currentPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(goalPose.getTranslation(), new Rotation2d())).getTranslation();
    m_targetHeading = robotToTargetTranslation.getAngle().plus(Rotation2d.k180deg);

    if (m_drivetrain.robotBehindHub()) {
      if (m_drivetrain.getAlliance() == Alliance.Red) {
        m_targetHeading = Rotation2d.k180deg;
      } else {
        m_targetHeading = Rotation2d.kZero;
      }
    }

    // Prediction horizon based on time-of-flight + small latency margin
    double timeOfFlight = m_timeOfFlightTable.get(currentPose.getTranslation().getDistance(m_goalPose.getTranslation()));
    final double kLatencyCompensation = 0.10; // seconds, tune as needed
    double predictionTime = timeOfFlight + kLatencyCompensation;

    // Predict future robot pose using field-relative speeds
    ChassisSpeeds fieldSpeeds = getFieldRelativeChassisSpeeds(m_drivetrain.getState().Speeds, currentPose);
    double futureX = currentPose.getX() + fieldSpeeds.vxMetersPerSecond * predictionTime;
    double futureY = currentPose.getY() + fieldSpeeds.vyMetersPerSecond * predictionTime;
    Rotation2d futureRotation = currentPose.getRotation().plus(new Rotation2d(fieldSpeeds.omegaRadiansPerSecond * predictionTime));
    Pose2d futurePose = new Pose2d(new Translation2d(futureX, futureY), futureRotation);

    // Compute the heading to the target from the predicted pose
    Translation2d robotToTargetTranslationFuture = poseInverse(new Pose2d(futurePose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(goalPose.getTranslation(), new Rotation2d())).getTranslation();
    Rotation2d futureTargetHeading = robotToTargetTranslationFuture.getAngle().plus(Rotation2d.k180deg);

    // Use controller to compute rotational velocity to intercept the future heading
    double thetaVelocity = m_rotationController.calculate(currentPose.getRotation().getRadians(), futureTargetHeading.getRadians());

    // Drive inputs (robot-relative from suppliers)
    double xVel = m_xVelocitySupplier.getAsDouble();
    double yVel = m_yVelocitySupplier.getAsDouble();
    if (new Translation2d(xVel, yVel).getNorm() < 0.2) {
      xVel = 0.0;
      yVel = 0.0;
    }

    // Build chassis speeds: convert robot-relative requested translation into field frame
    ChassisSpeeds CS = new ChassisSpeeds(xVel, yVel, 0.0);
    CS = ChassisSpeeds.fromFieldRelativeSpeeds(CS, currentPose.getRotation());
    CS = new ChassisSpeeds(CS.vxMetersPerSecond, CS.vyMetersPerSecond, thetaVelocity);

    m_drivetrain.setControl(m_driveRequest.withSpeeds(CS));

    Logger.recordOutput("Drivetrain/DriveToPose/ChassisSpeeds", CS);
    Logger.recordOutput("Drivetrain/RotateToPose/predictionTime", predictionTime);
    Logger.recordOutput("Drivetrain/RotateToPose/futurePose", futurePose);
    Logger.recordOutput("Drivetrain/RotateToPose/futureTargetHeading", futureTargetHeading.getDegrees());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds roboSpeed, Pose2d pose) {
    return new ChassisSpeeds(
        roboSpeed.vxMetersPerSecond * pose.getRotation().getCos()
                    - roboSpeed.vyMetersPerSecond * pose.getRotation().getSin(),
                    roboSpeed.vyMetersPerSecond * pose.getRotation().getCos()
                    + roboSpeed.vxMetersPerSecond * pose.getRotation().getSin(),
                    roboSpeed.omegaRadiansPerSecond);
    }


  private Pose2d poseInverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }
}
