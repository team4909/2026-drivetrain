package frc.robot.subsystems.turret;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretTrackPose extends Command{
    private Turret m_turret;
    private Pose2d m_goalPose;
    private ProfiledPIDController m_rotationalController;
    private Supplier<Pose2d> m_robotPoseSupplier;

    public TurretTrackPose(Turret turret, Pose2d goalPose, Supplier<Pose2d> robotPoseSupplier){
        m_turret = turret;
        m_goalPose = goalPose;

    // Initialize rotational controller to avoid NPE. Fine-tune gains/constraints as needed.
    // m_rotationalController = new ProfiledPIDController(4.0, 0.0, 0.0,
    //     new TrapezoidProfile.Constraints(10 * Math.PI, 10 * Math.PI));
    // allow wrapping around -pi..pi
    // m_rotationalController.enableContinuousInput(-Math.PI, Math.PI);

        m_robotPoseSupplier = robotPoseSupplier;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // m_rotationalController.setTolerance(Units.degreesToRadians(1.0));
        // Pose2d currentRobotPose = m_robotPoseSupplier.get();
        // Translation2d robotToTargetTranslation = poseInverse(new Pose2d(currentRobotPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(m_goalPose.getTranslation(), new Rotation2d())).getTranslation();
        // Rotation2d targetHeading = robotToTargetTranslation.getAngle().rotateBy(Rotation2d.k180deg);
        // // m_rotationalController.setTolerance(Units.degreesToRadians(1.0));
        // // prime controller with current turret angle as initial state
        // // Note: if turret position units differ, adapt accordingly.
        double currentTurretDeg = m_turret.getTurretPosition() * 360.0;
        // double currentTurretRad = Units.degreesToRadians(currentTurretDeg);
        // m_rotationalController.reset(currentTurretRad);
        // m_rotationalController.setGoal(targetHeading.getRadians());
    }

    @Override
    public void execute() {
        //         Pose2d currentRobotPose = m_robotPoseSupplier.get();
        // Translation2d robotToTargetTranslation = poseInverse(new Pose2d(currentRobotPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(m_goalPose.getTranslation(), new Rotation2d())).getTranslation();
        // Rotation2d targetHeading = robotToTargetTranslation.getAngle();
        // if (robotBehindTarget(currentRobotPose, m_goalPose)) {
        //     if(currentRobotPose.getRotation().getDegrees() >= 0) {
        //     m_turret.setDegrees(MathUtil.clamp(-targetHeading.getDegrees()  + currentRobotPose.getRotation().getDegrees() - 180, -90,90));
        //     Logger.recordOutput("Turret/targetHeading", -targetHeading.getDegrees() + currentRobotPose.getRotation().getDegrees() - 180);

        //     }
        //     else {
        //     m_turret.setDegrees(-MathUtil.clamp(targetHeading.getDegrees()  - currentRobotPose.getRotation().getDegrees() - 180, -90,90));
        //     Logger.recordOutput("Turret/targetHeading", targetHeading.getDegrees() - currentRobotPose.getRotation().getDegrees() - 180);

        //     }
        // }
        // else {        
        //     if(currentRobotPose.getRotation().getDegrees() >= 0){
        //         m_turret.setDegrees(MathUtil.clamp(-targetHeading.getDegrees()  + currentRobotPose.getRotation().getDegrees() - 180, -90,90));
        //         Logger.recordOutput("Turret/targetHeading", -targetHeading.getDegrees() + currentRobotPose.getRotation().getDegrees() - 180);

        //     }
        //     else{
        //         m_turret.setDegrees(-MathUtil.clamp(targetHeading.getDegrees()  - currentRobotPose.getRotation().getDegrees() - 180, -90,90));
        //         Logger.recordOutput("Turret/targetHeading", targetHeading.getDegrees() - currentRobotPose.getRotation().getDegrees() - 180);

        //     }
        // }
        // Logger.recordOutput("Turret/robotBehindTarget", robotBehindTarget(currentRobotPose, m_goalPose));

        Pose2d robotPose = m_robotPoseSupplier.get();

    // If robot is behind the hub, aim at the nearest corner instead of the
    // provided goal pose. `HUB_POSE` and `FIELD_CORNERS` are defined below.
    Pose2d targetPose = robotBehindHub(robotPose, HUB_POSE) ? robotPose.nearest(FIELD_CORNERS) : m_goalPose;

    // Compute vector from robot to target in field coordinates
    Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());

        // Angle from field +X axis to target (degrees)
        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        Logger.recordOutput("Turret/angleToTarget", angleToTargetFieldDeg);

        // Robot heading in degrees (field frame)
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        Logger.recordOutput("Turret/robotHeading", robotHeadingDeg);


    // Relative angle (signed shortest) from robot forward to target
    // No extra 180-degree offset — wrapDegrees already gives the minimal signed rotation
    double relativeDeg = wrapDegrees(angleToTargetFieldDeg - robotHeadingDeg);

    // Turret coordinate may be mirrored relative to our angle math depending on
    // motor/encoder wiring and convention. If the turret is rotating the
    // opposite direction of the shortest path, invert the sign here.
    // To make the turret track from the opposite side of the robot (flip
    // 180 degrees), add 180 to the relative angle before wrapping.
    double flippedRelative = wrapDegrees(relativeDeg + 180.0);
    // Clamp to turret travel limits and command (invert sign for motor
    // convention as before).
    double turretCmdDeg = MathUtil.clamp(-flippedRelative, -90.0, 90.0);
    m_turret.setDegrees(turretCmdDeg);

    Logger.recordOutput("Turret/flippedRelative", flippedRelative);

        Logger.recordOutput("Turret/targetHeading", turretCmdDeg);
        Logger.recordOutput("Turret/robotBehindTarget", robotBehindTarget(robotPose, m_goalPose));

    }

    // Field corner & hub definitions (placeholder coordinates — adjust to your field origin/units)
    private static final List<Pose2d> FIELD_CORNERS = List.of(
        new Pose2d(0, aprilTagLayout.getFieldWidth() - 1.5, new Rotation2d()),
        new Pose2d(0, 1.5, new Rotation2d())
        // new Pose2d(8.23, 5.49, new Rotation2d()),
        // new Pose2d(0.0, 5.49, new Rotation2d())
    );

    // Example hub position (center of field) — change if your coordinate system differs
    private static final Pose2d HUB_POSE = new Pose2d(
                aprilTagLayout.getTagPose(26).orElse(new Pose3d()).toPose2d().transformBy(new Transform2d(new Translation2d(-0.6,0), Rotation2d.k180deg)).getTranslation(),
            new Rotation2d());

    private boolean robotBehindHub(Pose2d robotPose, Pose2d hubPose) {
        // Simple X-based check; adapt this to your field's coordinate convention if needed.
        return robotPose.getX() > hubPose.getX();
    }

    /**
     * Normalize degrees to the interval (-180, 180].
     */
    private double wrapDegrees(double deg) {
        while (deg <= -180.0) {
            deg += 360.0;
        }
        while (deg > 180.0) {
            deg -= 360.0;
        }
        return deg;
    }


    private Pose2d poseInverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
      }

    private boolean robotBehindTarget(Pose2d robotPose, Pose2d targetPose) {
        return robotPose.getX() > targetPose.getX();
    }


    

    
    
}