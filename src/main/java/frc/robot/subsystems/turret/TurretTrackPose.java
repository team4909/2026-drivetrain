package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        Pose2d currentRobotPose = m_robotPoseSupplier.get();
        Translation2d robotToTargetTranslation = poseInverse(new Pose2d(currentRobotPose.getTranslation(), new Rotation2d())).transformBy(new Transform2d(m_goalPose.getTranslation(), new Rotation2d())).getTranslation();
        Rotation2d targetHeading = robotToTargetTranslation.getAngle();
        if(currentRobotPose.getRotation().getDegrees() >= 0){
            m_turret.setDegrees(MathUtil.clamp(-targetHeading.getDegrees()  + currentRobotPose.getRotation().getDegrees() - 180, -90,90));
            Logger.recordOutput("Turret/targetHeading", -targetHeading.getDegrees() + currentRobotPose.getRotation().getDegrees() - 180);

        }
        else{
            m_turret.setDegrees(-MathUtil.clamp(targetHeading.getDegrees()  - currentRobotPose.getRotation().getDegrees() - 180, -90,90));
            Logger.recordOutput("Turret/targetHeading", targetHeading.getDegrees() - currentRobotPose.getRotation().getDegrees() - 180);

        }

        // Logger.recordOutput("Turret/targetHeading", -targetHeading.getDegrees() + currentRobotPose.getRotation().getDegrees() - 180);
        // Logger.recordOutput("Turret/RobotPoseTrans", robotToTargetTranslation);
        // // Logger.recordOutputs("Turret/turretAngleideal")
        // m_turret.setDegrees(MathUtil.clamp(-targetHeading.getDegrees()  + currentRobotPose.getRotation().getDegrees() - 180, -90,90));
        // Rotation2d turretHeading = new Rotation2d(Units.degreesToRadians(m_turret.getTurretPosition()*360));

        

    }


    private Pose2d poseInverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(
            pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
      }

    
    
}
