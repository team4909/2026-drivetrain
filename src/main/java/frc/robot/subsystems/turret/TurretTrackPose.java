package frc.robot.subsystems.turret;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.Comparator;
import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.function.DoubleSupplier;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShootingParameters;

public class TurretTrackPose extends Command{
    private Turret m_turret;
    private Supplier<Pose2d> m_robotPoseSupplier;
    private DoubleSupplier m_requiredAngle;

    public TurretTrackPose(DoubleSupplier requiredAngle, Supplier<Pose2d> robotPoseSupplier, Turret turret){
        m_turret = turret;
        m_requiredAngle = requiredAngle;
        m_robotPoseSupplier = robotPoseSupplier;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    Pose2d robotPose = m_robotPoseSupplier.get();
    double angleToTargetFieldDeg = m_requiredAngle.getAsDouble();

    Logger.recordOutput("Turret/angleToTarget", angleToTargetFieldDeg);

    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    Logger.recordOutput("Turret/robotHeading", robotHeadingDeg);

    double relativeDeg = wrapDegrees(angleToTargetFieldDeg - robotHeadingDeg);

    double flippedRelative = wrapDegrees(relativeDeg + 180.0);

    double turretCmdDeg = MathUtil.clamp(-flippedRelative, -105.0, 105.0);
    m_turret.setDegrees(turretCmdDeg);

    Logger.recordOutput("Turret/flippedRelative", flippedRelative);

    Logger.recordOutput("Turret/targetHeading", turretCmdDeg);
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
    
}