// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.RotateToPose;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShootingCalculator;
import frc.robot.subsystems.shooter.ShootingParameters;
// import frc.robot.subsystems.turret.Turret;
// import frc.robot.subsystems.turret.TurretIOTalonFX;
// import frc.robot.subsystems.turret.TurretTrackPose;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity
        private SwerveRequest.ApplyRobotSpeeds m_drive;
        // LoggedNetworkNumber tunableNumber = new
        // LoggedNetworkNumber("/Tuning/MyTunableNumber", 0.0);

        // private Translation2d m_hub = aprilTagLayout.getTagPose(26).orElse(new
        // Pose3d()).toPose2d().transformBy(new Transform2d(new Translation2d(-0.6,0),
        // Rotation2d.k180deg)).getTranslation();

        // private final LoggedNetworkNumber Translation_P = new
        // LoggedNetworkNumber("/Tuning/Elevator/L1Setpoint", 10);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);

        public final CommandSwerveDrivetrain s_Drivetrain = TunerConstants.createDrivetrain();
        private final Shooter s_Shooter;
        private final Hood s_Hood;
        private final Indexer s_Indexer;
        // private final Turret s_Turret;
        private final Intake s_Intake;
        private final SendableChooser<Command> m_chooser;
        private final SendableChooser<Boolean> m_mirror;
        private ShootingCalculator m_shootingCalculator = new ShootingCalculator(s_Drivetrain);
        private ShootingParameters m_shootingParameters;
        private final Vision s_Vision;

        private final Command oscillate;

        public RobotContainer() {
                // if (DriverStation.getAlliance().isPresent() &&
                // DriverStation.getAlliance().get() == Alliance.Red){
                // m_hub = m_hub.rotateAround(new
                // Translation2d(aprilTagLayout.getFieldLength()/2,
                // aprilTagLayout.getFieldWidth()/2), Rotation2d.k180deg);
                // }

                m_drive = new SwerveRequest.ApplyRobotSpeeds();
                s_Shooter = new Shooter(new ShooterIOTalonFX());
                s_Indexer = new Indexer(new IndexerIOTalonFX());
                // Create intake and give it a reference to the indexer so combined
                // intake+index commands can be created inside the intake subsystem.
                s_Intake = new Intake(new IntakeIOTalonFX());
                s_Hood = new Hood(new HoodIOTalonFX());

                oscillate = Commands.repeatingSequence(
                                                Commands.race(s_Intake.bottomOscillate(), Commands.waitSeconds(0.15)),
                                                Commands.race(s_Intake.bumpAndRun(), Commands.waitSeconds(0.15)));

                NamedCommands.registerCommand("IntakeDownGo", s_Intake.intakeAndExtend());
                NamedCommands.registerCommand("IntakeUpStop", s_Intake.stowAndStop());
                NamedCommands.registerCommand("IntakeStop", s_Intake.stop());
                NamedCommands.registerCommand("IntakeGo", s_Intake.intake());
                NamedCommands.registerCommand("IntakeZeroDown", s_Intake.reZeroDown());
                NamedCommands.registerCommand("IntakeOscillate", oscillate);
                NamedCommands.registerCommand("HoodDown", s_Hood.retractHood());
                NamedCommands.registerCommand("HoodUp", s_Hood.extendHood());
                NamedCommands.registerCommand("HoodInterp", s_Hood
                                .goTo(() -> m_shootingParameters.calculate(s_Drivetrain.getHubCenter()).hoodAngle()));
                NamedCommands.registerCommand("ShootAndIndex", Commands.parallel(s_Shooter
                                .shoot(() -> m_shootingParameters.calculate(s_Drivetrain.getHubCenter()).rpm()),
                                s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()));
                NamedCommands.registerCommand("Pass",
                                Commands.parallel(s_Shooter.shoot(m_shootingCalculator::getShooterSpeed),
                                                s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()));
                NamedCommands.registerCommand("ShootIndexStop", Commands.parallel(s_Shooter.stop(), s_Indexer.stop()));
                NamedCommands.registerCommand("ShooterSpinUp", s_Shooter.shoot(() -> m_shootingParameters.calculate(s_Drivetrain.getHubCenter()).rpm()));
                NamedCommands.registerCommand("IndexerFeed", s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly());
                NamedCommands.registerCommand("IndexerStop", s_Indexer.stop());
                NamedCommands.registerCommand("RotateToHub", new RotateToPose(s_Drivetrain, () -> -joystick.getLeftY() * MaxSpeed, ()-> -joystick.getLeftX() * MaxSpeed));
                
                s_Vision = new Vision(s_Drivetrain::addVisionMeasurement,
                                new VisionIOPhotonVision("back-left-cam", new Transform3d(new Translation3d(
                                                Units.inchesToMeters(-12.772144),
                                                Units.inchesToMeters(5.092677),
                                                Units.inchesToMeters(13.442920)),
                                                new Rotation3d(
                                                                Units.degreesToRadians(0.0),
                                                                Units.degreesToRadians(-20),
                                                                Units.degreesToRadians(180)))),
                                new VisionIOPhotonVision("back-left-cam1", new Transform3d(new Translation3d(
                                                Units.inchesToMeters(-12.772144),
                                                Units.inchesToMeters(5.092677),
                                                Units.inchesToMeters(13.442920)),
                                                new Rotation3d(
                                                                Units.degreesToRadians(0.0),
                                                                Units.degreesToRadians(-20),
                                                                Units.degreesToRadians(180)))),
                                new VisionIOPhotonVision("back-right-cam", new Transform3d(new Translation3d(
                                                Units.inchesToMeters(-12.966111),
                                                Units.inchesToMeters(-5.092667),
                                                Units.inchesToMeters(13.454091)),
                                                new Rotation3d(
                                                                Units.degreesToRadians(0.0),
                                                                Units.degreesToRadians(-15),
                                                                Units.degreesToRadians(180)))),
                                new VisionIOPhotonVision("back-right-cam1", new Transform3d(new Translation3d(
                                                Units.inchesToMeters(-12.966111),
                                                Units.inchesToMeters(-5.092667),
                                                Units.inchesToMeters(13.454091)),
                                                new Rotation3d(
                                                                Units.degreesToRadians(0.0),
                                                                Units.degreesToRadians(-15),
                                                                Units.degreesToRadians(180)))));

                m_shootingParameters = new ShootingParameters(s_Drivetrain);

                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                        AutoBuilder.configure(
                                        () -> s_Drivetrain.getState().Pose,
                                        s_Drivetrain::resetPose,
                                        s_Drivetrain::getRobotRelativeSpeeds,
                                        (speeds, feedforwards) -> s_Drivetrain.setControl(m_drive.withSpeeds(speeds)),
                                        // this.ChassisSpeeds(Translation2d.driveVelocity.getX(),
                                        // Translation2d.driveVelocity.getY(), thetaVelocity),
                                        new PPHolonomicDriveController(
                                                        new PIDConstants(5.0, 0.0, 0.0),
                                                        new PIDConstants(5.0, 0.0, 0.0)),
                                        config,
                                        () -> {
                                                var alliance = DriverStation.getAlliance();
                                                if (alliance.isPresent()) {
                                                        return alliance.get().equals(Alliance.Red);
                                                }
                                                return false;
                                        }, s_Drivetrain);
                } catch (Exception e) {
                        // Handle exception as needed
                        e.printStackTrace();
                }
                m_chooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", m_chooser);

                m_mirror = new SendableChooser<Boolean>();
                m_mirror.addOption("Normal Auto", false);
                m_mirror.addOption("Mirror Auto", true);
                SmartDashboard.putData("Mirror Auto", m_mirror);

                configureBindings();
        }

        private void configureBindings() {

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                s_Drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                s_Drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                     // forward
                                                                                                                     // with
                                                                                                                     // negative
                                                                                                                     // Y
                                                                                                                     // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));

                s_Hood.setDefaultCommand(s_Hood.retractHood()); // USE THIS
                s_Shooter.setDefaultCommand(s_Shooter.stop()); // USE THIS`
                s_Indexer.setDefaultCommand(s_Indexer.stop()); // USE THIS

                // s_Hood.setDefaultCommand(s_Hood.tunableShot()); // TUNING

                joystick.start().onTrue(s_Drivetrain.runOnce(() -> s_Drivetrain.seedFieldCentric()));

                joystick.povRight().whileTrue(Commands.race(Commands.waitSeconds(0.75), s_Hood.runBack())
                                .andThen(s_Hood.zeroButton()));

                joystick.y().whileTrue(s_Intake.reZero());

                joystick.a().whileTrue(s_Shooter.shoot(() -> 10000)).onFalse(s_Shooter.stop());

                joystick.x().whileTrue(Commands.parallel(s_Indexer.outtake(), s_Intake.outtake())).onFalse(Commands.parallel(s_Intake.stop(), s_Indexer.stop()));

                joystick.back().whileTrue(s_Intake.reZero());

                joystick.leftTrigger().whileTrue(Commands.parallel(s_Indexer.kickerPull(), s_Intake.intake())).onFalse(Commands.parallel(s_Intake.stop(), s_Indexer.stop()));


                //TUNING
                // joystick.rightTrigger().whileTrue(Commands.parallel(
                // s_Shooter.tuningShoot(),
                // Commands.sequence(Commands.waitSeconds(3), s_Indexer.tuningShoot())
                // )).onFalse(Commands.parallel(
                // s_Shooter.stop(),
                // s_Indexer.stop()
                // ));

                joystick.leftBumper()
                                .onTrue(s_Intake.Extend());

                // passing shot
                joystick.povLeft().whileTrue(
                                Commands.parallel(
                                                s_Hood.goTo(m_shootingCalculator::getHoodPosition),
                                                s_Shooter.shoot(m_shootingCalculator::getShooterSpeed),
                                                new RotateToPose(s_Drivetrain, () -> -joystick.getLeftY() * MaxSpeed, ()-> -joystick.getLeftX() * MaxSpeed),
                                                s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()))
                                .onFalse(
                                                Commands.parallel(
                                                                s_Shooter.shoot(() -> 30),
                                                                s_Indexer.stop(),
                                                                s_Intake.Extend(),
                                                                Commands.race(Commands.waitSeconds(0.75),
                                                                                s_Hood.runBack())
                                                                                .andThen(s_Hood.zeroButton())
                                                                                .andThen(s_Hood.retractHood())

                                                ));

                // scoring shot
                joystick.rightTrigger().whileTrue(
                                Commands.parallel(
                                                s_Hood.goTo(() -> m_shootingParameters
                                                                .calculate(s_Drivetrain.getHubCenter()).hoodAngle()),
                                                s_Shooter.shoot(() -> m_shootingParameters
                                                                .calculate(s_Drivetrain.getHubCenter()).rpm()),
                                                new RotateToPose(s_Drivetrain, () -> -joystick.getLeftY() * MaxSpeed, ()-> -joystick.getLeftX() * MaxSpeed),
                                                s_Indexer.feed().beforeStarting(Commands.waitSeconds(0.51)))

                ).onFalse(
                                Commands.parallel(
                                                s_Shooter.stop(),
                                                s_Indexer.stop(),
                                                s_Intake.Extend(),
                                                Commands.race(Commands.waitSeconds(0.75),
                                                                s_Hood.runBack())
                                                                .andThen(s_Hood.zeroButton())
                                                                .andThen(s_Hood.retractHood())

                                ));

                //no cam shooting
                joystick.b().whileTrue(
                        Commands.parallel(
                                        s_Hood.goTo(() -> 0.8),
                                        s_Shooter.shoot(() -> 33.0),
                                        s_Indexer.feed().beforeStarting(Commands.waitSeconds(0.51))))
                .onFalse(
                        Commands.parallel(
                                        s_Shooter.stop(),
                                        s_Indexer.stop(),
                                        s_Intake.Extend(),
                                        Commands.race(Commands.waitSeconds(0.75),
                                                        s_Hood.runBack())
                                                        .andThen(s_Hood.zeroButton())
                                                        .andThen(s_Hood.retractHood()))
                );

                joystick.rightBumper()
                                .whileTrue(oscillate)
                                .onFalse(Commands.sequence(s_Intake.Extend()));

                s_Drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                if (m_mirror.getSelected() != null && m_mirror.getSelected().booleanValue()) {
                        return new PathPlannerAuto(m_chooser.getSelected().getName(), true);
                } 
                else {
                        return m_chooser.getSelected();
                }

        }
}
