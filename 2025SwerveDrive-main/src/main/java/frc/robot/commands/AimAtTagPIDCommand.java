package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionAim;
import frc.robot.Constants.VisionConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;

/**
 * Drives the robot toward the nearest AprilTag using odometry and field layout.
 * No vision required. Includes rotation control and smooth slowdown.
 */
public class AimAtTagPIDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionAim vision;
    private final Transform2d tagOffset;

    private Pose2d goalPose;
    private boolean hasTarget = false;

    public AimAtTagPIDCommand(Transform2d tagOffset, CommandSwerveDrivetrain drivetrain, VisionAim vision) {
        this.tagOffset = tagOffset;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        hasTarget = false;

        Pose2d robotPose = drivetrain.getPose();
        Optional<Pose2d> goalOpt = vision.computeNearestGoalPose(robotPose, tagOffset);

        if (goalOpt.isEmpty()) {
            System.out.println("[AimAtTagPID] No tags found in field layout.");
            return;
        }

        goalPose = goalOpt.get();
        hasTarget = true;

        System.out.printf("[AimAtTagPID] Target tag goal at (%.2f, %.2f) deg=%.1f%n",
                goalPose.getX(), goalPose.getY(), goalPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (!hasTarget) return;

        Pose2d currentPose = drivetrain.getPose();

        // Continuously visualize positions
        // drivetrain.getField().getObject("RobotPose").setPose(currentPose);
        // drivetrain.getField().getObject("GoalPose").setPose(goalPose);

        // Compute translational error in robot frame
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        Translation2d fieldError = new Translation2d(dx, dy);
        Translation2d robotError = fieldError.rotateBy(currentPose.getRotation().unaryMinus());

        double xErr = robotError.getX();
        double yErr = robotError.getY();

        // Proportional control
        double vx = VisionConstants.KP_XY * xErr;
        double vy = VisionConstants.KP_XY * yErr;

        // Orientation control (face tag)
        double headingError = MathUtil.angleModulus(
                goalPose.getRotation().minus(currentPose.getRotation()).getRadians());
        double omega = VisionConstants.KP_THETA * headingError;

        // Clamp speeds
        vx = MathUtil.clamp(vx, -VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);
        vy = MathUtil.clamp(vy, -VisionConstants.MAX_VY_M_PER_S, VisionConstants.MAX_VY_M_PER_S);
        omega = MathUtil.clamp(omega, -VisionConstants.MAX_OMEGA_RAD_PER_S, VisionConstants.MAX_OMEGA_RAD_PER_S);

        // Gradual slowdown near goal
        double distToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        if (distToGoal < 0.5) {
            double scale = MathUtil.clamp(distToGoal / 0.5, 0.2, 1.0);
            vx *= scale;
            vy *= scale;
        }

        drivetrain.setOperatorPerspectiveForward(Rotation2d.kZero);
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(vx, vy, omega))
                .withCenterOfRotation(new Translation2d()));
    }

    @Override
    public boolean isFinished() {
        if (!hasTarget) return true;
        Pose2d cur = drivetrain.getPose();
        double posDist = cur.getTranslation().getDistance(goalPose.getTranslation());
        double rotDist = Math.abs(goalPose.getRotation().minus(cur.getRotation()).getDegrees());
        return posDist < VisionConstants.GOAL_POS_EPS_M && rotDist < 5.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("[AimAtTagPID] " + (interrupted ? "Interrupted" : "Completed"));
    }
}
