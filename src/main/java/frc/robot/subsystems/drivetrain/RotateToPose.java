package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.net.DatagramSocket;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class RotateToPose extends Command{
    private CommandSwerveDrivetrain m_drivetrain;
    private Pose2d m_goalPose;
    private final ProfiledPIDController m_rotationController;
    private SwerveRequest.ApplyRobotSpeeds m_driveRequest;
    private Rotation2d m_targetHeading;

    private DoubleSupplier m_xVelocitySupplier;
    private DoubleSupplier m_yVelocitySupplier;

    public RotateToPose(CommandSwerveDrivetrain drivetrain, DoubleSupplier xVelocity, DoubleSupplier yVelocity) {
        m_drivetrain = drivetrain;

        m_xVelocitySupplier = xVelocity;
        m_yVelocitySupplier = yVelocity;

        m_goalPose = new Pose2d(drivetrain.getHubCenter(), new Rotation2d());
        m_rotationController = new ProfiledPIDController(10.0, 0.0, 0.01, new TrapezoidProfile.Constraints(100 * Math.PI/10*2, 100 * Math.PI/10*3));
        m_driveRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        addRequirements(drivetrain);
    }


    @Override
  public void initialize() {
    Pose2d initialPose = m_drivetrain.getState().Pose;

    Translation2d robotToTargetTranslation = poseInverse(new Pose2d(initialPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(m_goalPose.getTranslation(), new Rotation2d())).getTranslation();

    m_targetHeading = robotToTargetTranslation.getAngle();//.rotateBy(Rotation2d.k180deg);



    

    m_rotationController.setTolerance(Units.degreesToRadians(1.0));

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    ChassisSpeeds robotVelocity = getFieldRelativeChassisSpeeds(m_drivetrain.getState().Speeds, initialPose);

    m_rotationController.reset(
        initialPose.getRotation().getRadians(),
        robotVelocity.omegaRadiansPerSecond);
    
  }

  @Override
  public void execute() {

    Pose2d currentPose = m_drivetrain.getState().Pose;

    Translation2d robotToTargetTranslation = poseInverse(new Pose2d(currentPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(m_goalPose.getTranslation(), new Rotation2d())).getTranslation();
    m_targetHeading = robotToTargetTranslation.getAngle().plus(Rotation2d.k180deg);

    if(m_drivetrain.robotBehindHub()) {
      if (m_drivetrain.getAlliance() == Alliance.Red) {
        m_targetHeading = Rotation2d.k180deg;
      }
      else {
        m_targetHeading = Rotation2d.kZero;
      }
    }



    // double distanceToGoalPose =
    //     currentPose.getRotation().minus(m_targetHeading).getRadians();

    // double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);









    double thetaVelocity =
        m_rotationController.calculate(
                currentPose.getRotation().getRadians(), m_targetHeading.getRadians());


    // double thetaErrorAbsolute =
    //     Math.abs(currentPose.getRotation().minus(m_goalPose.getRotation()).getRadians());
    // if (thetaErrorAbsolute < m_thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Translation2d driveVelocity =
    //     new Pose2d(
    //             new Translation2d(),
    //             currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
    //         .transformBy(
    //             new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
    //         .getTranslation();
    double xVel = m_xVelocitySupplier.getAsDouble();// > 0.2 ? m_xVelocitySupplier.getAsDouble() : 0.0;
    double yVel = m_yVelocitySupplier.getAsDouble();// > 0.2 ? m_yVelocitySupplier.getAsDouble() : 0.0;

    // if(new Translation2d(xVel, yVel).getNorm() < 0.2) {
      xVel = 0.0;
      yVel = 0.0;
    // }

    ChassisSpeeds CS = new ChassisSpeeds(xVel,yVel, 0);//(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);

    CS = ChassisSpeeds.fromFieldRelativeSpeeds(CS, currentPose.getRotation());

    CS = new ChassisSpeeds(CS.vxMetersPerSecond, CS.vyMetersPerSecond, thetaVelocity);

    m_drivetrain.setControl(m_driveRequest.withSpeeds(CS));

    Logger.recordOutput("Drivetrain/DriveToPose/ChassisSpeeds", CS);

    // Logger.recordOutput("Drivetrain/DriveToPose/DistanceToGoalPose", distanceToGoalPose);
    // Logger.recordOutput(
    //     "Drivetrain/DriveToPose/Setpoint",
    //     new Pose2d(
    //         m_lastSetpointTranslation, new Rotation2d(m_thetaController.getSetpoint().position)));
    // Logger.recordOutput("Drivetrain/DriveToPose/Goal", m_goalPose);
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
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
      }
    

}
