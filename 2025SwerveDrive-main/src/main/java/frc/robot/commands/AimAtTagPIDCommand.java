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
 * No vision required.
 */
public class AimAtTagPIDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionAim vision;
    private final Transform2d robot2goal;

    private Pose2d goalPose;
    private boolean hasTarget = false;

    public AimAtTagPIDCommand(Transform2d robot2goal, CommandSwerveDrivetrain drivetrain, VisionAim vision) {
        this.robot2goal = robot2goal;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        hasTarget = false;

        Pose2d robotPose = drivetrain.getPose();
        Optional<Pose2d> goalOpt = vision.computeNearestGoalPose(robotPose, robot2goal);

        if (goalOpt.isEmpty()) {
            System.out.println("[AimAtTagPID] No tags found in field layout.");
            return;
        }

        goalPose = goalOpt.get();
        hasTarget = true;

        System.out.printf("[AimAtTagPID] Moving toward nearest tag goal at (%.2f, %.2f) deg=%.1f%n",
                goalPose.getX(), goalPose.getY(), goalPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (!hasTarget) return;

        Pose2d currentPose = drivetrain.getPose();

        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        Translation2d fieldError = new Translation2d(dx, dy);
        Translation2d robotError = fieldError.rotateBy(currentPose.getRotation().unaryMinus());

        double xErr = robotError.getX();
        double yErr = robotError.getY();

        if (Math.abs(xErr) < VisionConstants.XY_DEADBAND_M) xErr = 0.0;
        if (Math.abs(yErr) < VisionConstants.XY_DEADBAND_M) yErr = 0.0;

        double vx = MathUtil.clamp(VisionConstants.KP_XY * xErr,
                -VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);
        double vy = MathUtil.clamp(VisionConstants.KP_XY * yErr,
                -VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);

        drivetrain.setOperatorPerspectiveForward(Rotation2d.kZero);
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(vx, vy, 0))
                .withCenterOfRotation(new Translation2d()));
    }

    @Override
    public boolean isFinished() {
        if (!hasTarget) return true;
        Pose2d cur = drivetrain.getPose();
        double dist = cur.getTranslation().getDistance(goalPose.getTranslation());
        return dist < VisionConstants.GOAL_POS_EPS_M;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("[AimAtTagPID] " + (interrupted ? "Interrupted" : "Completed"));
    }
}
