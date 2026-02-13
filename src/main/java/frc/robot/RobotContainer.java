// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveToPose;
import frc.robot.subsystems.drivetrain.RotateToPose;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIOCandle;
import frc.robot.subsystems.led.Led.AnimationType;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private SwerveRequest.ApplyRobotSpeeds m_drive;
    LoggedNetworkNumber tunableNumber = new LoggedNetworkNumber("/Tuning/MyTunableNumber", 0.0);

    private final LoggedNetworkNumber Translation_P = new LoggedNetworkNumber("/Tuning/Elevator/L1Setpoint", 10);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter s_Shooter;
    private final Indexer s_Indexer;
    private final SendableChooser<Command> m_chooser;

    private final Vision s_Vision;
    private final Led s_Led;

    public RobotContainer() {
        m_drive = new SwerveRequest.ApplyRobotSpeeds();
        s_Shooter = new Shooter(new ShooterIOTalonFX());
        s_Indexer = new Indexer(new IndexerIOTalonFX());
        s_Led = new Led(new LedIOCandle());
        s_Vision = new Vision(drivetrain::addVisionMeasurement,
                    new VisionIOPhotonVision("back-left-cam", new Transform3d(new Translation3d(
                            Units.inchesToMeters(-10.92),
                            Units.inchesToMeters(10.92),
                            Units.inchesToMeters(8.709057)),
                            new Rotation3d(
                                    Units.degreesToRadians(0.0),
                                    Units.degreesToRadians(-61.87),
                                    Units.degreesToRadians(90+45)))),

                    new VisionIOPhotonVision("front-left-cam", new Transform3d(new Translation3d(
                            Units.inchesToMeters(10.92),
                            Units.inchesToMeters(10.92),
                            Units.inchesToMeters(8.709057)),
                            new Rotation3d(
                                    Units.degreesToRadians(0.0),
                                    Units.degreesToRadians(-61.87),
                                    Units.degreesToRadians(45)))),
                    new VisionIOPhotonVision("back-right-cam", new Transform3d(new Translation3d(
                            Units.inchesToMeters(-10.92),
                            Units.inchesToMeters(-10.92),
                            Units.inchesToMeters(8.709057)),
                            new Rotation3d(
                                    Units.degreesToRadians(0.0),
                                    Units.degreesToRadians(-61.87),
                                    Units.degreesToRadians(180+45)))),
                    new VisionIOPhotonVision("front-right-cam", new Transform3d(new Translation3d(
                            Units.inchesToMeters(10.92),
                            Units.inchesToMeters(-10.92),
                            Units.inchesToMeters(8.709057)),
                            new Rotation3d(
                                    Units.degreesToRadians(0.0),
                                    Units.degreesToRadians(-61.87),
                                    Units.degreesToRadians(270+45)))));

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> drivetrain.getState().Pose,
                    drivetrain::resetPose,
                    drivetrain::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> drivetrain.setControl(m_drive.withSpeeds(speeds)),
                    // this.ChassisSpeeds(Translation2d.driveVelocity.getX(),
                    // Translation2d.driveVelocity.getY(), thetaVelocity),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    }, drivetrain);
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_chooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));
        
        s_Led.setDefaultCommand(s_Led.setAnimation(Led.AnimationType.Off));
        
        joystick.a().whileTrue(s_Led.setAnimation(Led.AnimationType.Fire)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.b().whileTrue(s_Led.setAnimation(Led.AnimationType.Larson)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.x().whileTrue(s_Led.setAnimation(Led.AnimationType.SingleFade)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.y().whileTrue(s_Led.setAnimation(Led.AnimationType.RgbFade)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.leftBumper().whileTrue(s_Led.setAnimation(Led.AnimationType.Strobe)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.rightBumper().whileTrue(s_Led.setAnimation(Led.AnimationType.Twinkle)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.povUp().whileTrue(s_Led.setAnimation(Led.AnimationType.ColorFlow)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        joystick.povDown().whileTrue(s_Led.setAnimation(Led.AnimationType.TwinkleOff)).onFalse(s_Led.setAnimation(Led.AnimationType.Off));
        // joystick.rightTrigger().whileTrue(Commands.sequence(s_Shooter.shoot(),s_Indexer.feed())).onFalse(Commands.sequence(s_Shooter.stop(),s_Indexer.stop()));
        



        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))

        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //joystick.rightTrigger().onTrue(drivetrain.startLogger());
        //joystick.leftTrigger().onTrue(drivetrain.stopLogger());

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        // return new PathPlannerAuto("cut");

    }
}
