// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.RotateToPose;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIORevServoHub;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShootingCalculator;
import frc.robot.subsystems.shooter.ShootingParameters;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretTrackPose;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private SwerveRequest.ApplyRobotSpeeds m_drive;
//     LoggedNetworkNumber tunableNumber = new LoggedNetworkNumber("/Tuning/MyTunableNumber", 0.0);

    private Translation2d m_hub = aprilTagLayout.getTagPose(26).orElse(new Pose3d()).toPose2d().transformBy(new Transform2d(new Translation2d(-0.6,0), Rotation2d.k180deg)).getTranslation();

//     private final LoggedNetworkNumber Translation_P = new LoggedNetworkNumber("/Tuning/Elevator/L1Setpoint", 10);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain s_Drivetrain = TunerConstants.createDrivetrain();
    private final Shooter s_Shooter;
    private final Hood s_Hood;
    private final Indexer s_Indexer;
    private final Turret s_Turret;
    private final Intake s_Intake;
    private final SendableChooser<Command> m_chooser;
//     private ShootingCalculator m_shootingCalculator = new ShootingCalculator(drivetrain);
    private ShootingParameters m_shootingParameters;
    private final Vision s_Vision;

    public RobotContainer() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            m_hub = m_hub.rotateAround(new Translation2d(aprilTagLayout.getFieldLength()/2, aprilTagLayout.getFieldWidth()/2), Rotation2d.k180deg);
    }


        m_drive = new SwerveRequest.ApplyRobotSpeeds();
        s_Shooter = new Shooter(new ShooterIOTalonFX());
        s_Indexer = new Indexer(new IndexerIOTalonFX());
        // Create intake and give it a reference to the indexer so combined
        // intake+index commands can be created inside the intake subsystem.
        s_Intake = new Intake(new IntakeIOTalonFX());
        s_Hood = new Hood(new HoodIORevServoHub());
        // s_Shooter = new Shooter(new ShooterIOTalonFX());
        s_Turret = new Turret(new TurretIOTalonFX());

        NamedCommands.registerCommand("IntakeDownGo", s_Intake.intakeAndExtend());
        NamedCommands.registerCommand("IntakeUpStop", s_Intake.stowAndStop());
        NamedCommands.registerCommand("IntakeStop", s_Intake.stop());
        NamedCommands.registerCommand("IntakeGo", s_Intake.intake());
        NamedCommands.registerCommand("IntakeZeroDown", s_Intake.reZeroDown());
        NamedCommands.registerCommand("IntakeOscillate", Commands.repeatingSequence(Commands.race(s_Intake.Extend(), Commands.waitSeconds(0.1)), Commands.race(s_Intake.stow(), Commands.waitSeconds(0.1))));
        NamedCommands.registerCommand("HoodDown", s_Hood.retractHood());
        NamedCommands.registerCommand("HoodUp", s_Hood.extendHood());
        NamedCommands.registerCommand("HoodInterp", s_Hood.goTo(() -> m_shootingParameters.calculate(m_hub).hoodAngle()));
        NamedCommands.registerCommand("ShootAndIndex",  Commands.parallel(s_Shooter.shoot(() -> m_shootingParameters.calculate(m_hub).rpm()), s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()));
        NamedCommands.registerCommand("Pass", Commands.parallel(s_Shooter.shoot(() -> 60), s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()));
        NamedCommands.registerCommand("ShootIndexStop", Commands.parallel(s_Shooter.stop(),s_Indexer.stop()));

        s_Vision = new Vision(s_Drivetrain::addVisionMeasurement,
                new VisionIOPhotonVision("back-left-cam", new Transform3d(new Translation3d(
                        Units.inchesToMeters(-9.88538),
                        Units.inchesToMeters(10.762213),
                        Units.inchesToMeters(8.314941)), // 682965
                        new Rotation3d(
                                Units.degreesToRadians(0.0),
                                Units.degreesToRadians(-20),
                                Units.degreesToRadians(90.0 + 65)))), //90.0 + 65
                // new VisionIOPhotonVision("back-center-cam", new Transform3d(new Translation3d(
                //         Units.inchesToMeters(-11.935828),
                //         Units.inchesToMeters(11.338099),
                //         Units.inchesToMeters(14.3875)),
                //         new Rotation3d(
                //                 Units.degreesToRadians(7.435472),
                //                 Units.degreesToRadians(-30),
                //                 Units.degreesToRadians(192.5)
                //         ))),
                new VisionIOPhotonVision("left-cam", new Transform3d(new Translation3d(
                        Units.inchesToMeters(-6.748188),
                        Units.inchesToMeters(12.487542),
                        Units.inchesToMeters(8.315411)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-20),
                                Units.degreesToRadians(90)
                        ))),
                new VisionIOPhotonVision("left-cam1", new Transform3d(new Translation3d(
                        Units.inchesToMeters(-6.748188), //-9.24006310
                        Units.inchesToMeters(12.487542), //10.072250
                        Units.inchesToMeters(8.315411)), //15.539060
                        new Rotation3d(
                                Units.degreesToRadians(0), //5.6737923
                                Units.degreesToRadians(-20), //-21
                                Units.degreesToRadians(90) //69.656
                        ))),
                new VisionIOPhotonVision("right-cam", new Transform3d(new Translation3d(
                        Units.inchesToMeters(-6.748188),
                        Units.inchesToMeters(-12.487542),
                        Units.inchesToMeters(8.315411)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-20),
                                Units.degreesToRadians(270)
                        ))),
                new VisionIOPhotonVision("right-cam1", new Transform3d(new Translation3d(
                        Units.inchesToMeters(-6.748188),
                        Units.inchesToMeters(-12.487542),
                        Units.inchesToMeters(8.315411)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-20),
                                Units.degreesToRadians(270) //69.656
                        ))),

                // new VisionIOPhotonVision("front-left-cam", new Transform3d(new Translation3d(
                // Units.inchesToMeters(10.92),
                // Units.inchesToMeters(10.92),
                // Units.inchesToMeters(8.682965)),
                // new Rotation3d(
                // Units.degreesToRadians(0.0),
                // Units.degreesToRadians(-61.87),
                // Units.degreesToRadians(45)))),
                new VisionIOPhotonVision("back-right-cam", new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-9.888875),
                                Units.inchesToMeters(-10.761876),
                                Units.inchesToMeters(8.314941)),
                        new Rotation3d(
                                Units.degreesToRadians(0.0),
                                Units.degreesToRadians(-20),
                                Units.degreesToRadians(270.0 - 65)))));

        m_shootingParameters = new ShootingParameters(s_Drivetrain);
        // new VisionIOPhotonVision("front-right-cam", new Transform3d(new
        // Translation3d(
        // Units.inchesToMeters(10.92),
        // Units.inchesToMeters(-10.92),
        // Units.inchesToMeters(8.682965)),
        // new Rotation3d(
        // Units.degreesToRadians(0.0),
        // Units.degreesToRadians(-61.87),
        // Units.degreesToRadians(270+45)))));

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

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        s_Drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                s_Drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));
        // s_Hood.setDefaultCommand(s_Hood.goTo(m_shootingCalculator::getHoodPosition));

        s_Hood.setDefaultCommand(new ConditionalCommand(s_Hood.extendHood(), 
        s_Hood.goTo((() -> m_shootingParameters.calculate(m_hub).hoodAngle())), 
        s_Drivetrain::robotBehindHub));


        // s_Hood.setDefaultCommand(s_Hood.goTo(() -> m_shootingParameters.calculate(m_hub).hoodAngle()));
        // s_Hood.setDefaultCommand(s_Hood.tunableShot());
        joystick.start().onTrue(s_Drivetrain.runOnce(() -> s_Drivetrain.seedFieldCentric()));
        // joystick.x().whileTrue(s_Hood.extendHood());
        // joystick.y().whileTrue(s_Hood.retractHood());
        joystick.povRight().whileTrue(s_Hood.retractHood());
        joystick.y().whileTrue(s_Intake.reZero());
        // joystick.rightBumper().onTrue(s_Hood.tunableShot());
        joystick.a().whileTrue(s_Indexer.notintake()).onFalse(s_Indexer.stop());
        joystick.back().whileTrue(s_Intake.reZero());

        joystick.leftTrigger().whileTrue(s_Intake.intakePID()).onFalse(s_Intake.stop());

        // joystick.rightTrigger().whileTrue(Commands.parallel(
        //         s_Shooter.shoot(m_shootingCalculator::getShooterSpeed),
        //         Commands.sequence(Commands.waitSeconds(1), s_Indexer.feed()))).onFalse(Commands.parallel(
        //                 s_Shooter.stop(),
        //                 s_Indexer.stop()));

        // joystick.rightTrigger().whileTrue(Commands.parallel(
        //         s_Shooter.tuningShoot(),
        //         Commands.sequence(Commands.waitSeconds(4), s_Indexer.feed())
        //         )).onFalse(Commands.parallel(
        //                 s_Shooter.stop(),
        //                 s_Indexer.stop()
        //         ));

        joystick.rightTrigger().whileTrue(new ConditionalCommand(
                Commands.parallel(s_Shooter.shoot(() -> 100), s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()),
                Commands.parallel(s_Shooter.shoot(() -> m_shootingParameters.calculate(m_hub).rpm()), s_Indexer.feed().onlyIf(s_Shooter::atSpeed).repeatedly()),
                // Condition: true when robot is behind the hub (X greater than hub X)
                s_Drivetrain::robotBehindHub))
                .onFalse(Commands.parallel(
                        s_Shooter.stop(),
                        s_Indexer.stop()
                ));
        
        
                // joystick.rightTrigger().whileTrue(Commands.parallel(
                // s_Shooter.shoot(() -> m_shootingParameters.calculate(m_hub).rpm()),
                // Commands.sequence(Commands.waitSeconds(1), s_Indexer.feed()))).onFalse(Commands.parallel(
                //         s_Shooter.stop(),
                //         s_Indexer.stop()));

        // joystick.a().whileTrue(new RotateToPose(s_Drivetrain, (aprilTagLayout.getTagPose(26).orElse(new Pose3d())
        //         .toPose2d().transformBy(new Transform2d(new Translation2d(-0.6, 0), Rotation2d.kZero)))));
        // joystick.leftTrigger().whileTrue(s_Indexer.feed()).onFalse(s_Indexer.stop());
        s_Turret.setDefaultCommand(new TurretTrackPose(() -> m_shootingParameters.calculate(m_hub).turretAngle().getDegrees(), () -> s_Drivetrain.getState().Pose, s_Turret, m_hub));
        // joystick.x().whileTrue(new TurretTrackPose(s_Turret, m_hub, ()->
        // drivetrain.getState().Pose));
        // joystick.a().whileTrue(new RotateToPose(drivetrain,
        // (aprilTagLayout.getTagPose(18).orElse(new
        // Pose3d()).toPose2d().transformBy(new Transform2d(new Translation2d(0.4,0),
        // Rotation2d.kZero)))));

        // joystick.rightTrigger().whileTrue(Commands.sequence(s_Shooter.shoot(),s_Indexer.feed())).onFalse(Commands.sequence(s_Shooter.stop(),s_Indexer.stop()));

        joystick.b().whileTrue(s_Drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))

        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // // joystick.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.rightTrigger().onTrue(drivetrain.startLogger());
        // joystick.leftTrigger().onTrue(drivetrain.stopLogger());

        // // reset the field-centric heading on left bumper press
        // extend intake on left bumper press
       joystick.leftBumper()
                .onTrue(s_Intake.Extend());
                // .onFalse(Commands.sequence(s_Intake.stow()));
        joystick.x()
                .onTrue(s_Intake.stow());
                // .onFalse(Commands.sequence(s_Intake.stow()));
        joystick.povLeft().onTrue(s_Intake.bump()).onFalse(s_Intake.Extend());
        joystick.rightBumper()
                // .whileTrue(Commands.sequence(s_Intake.intakeAndExtend(), s_Intake.stowAndStop()).repeatedly())
                .whileTrue(Commands.repeatingSequence(Commands.race(s_Intake.intakeAndExtend(), Commands.waitSeconds(0.1)), Commands.race(s_Intake.bumpAndRun(), Commands.waitSeconds(0.1))))
                .onFalse(Commands.sequence(s_Intake.Extend(), s_Intake.stop()));

        s_Drivetrain.registerTelemetry(logger::telemeterize);
    }
        
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        // return new PathPlannerAuto("cut");

    }
}
