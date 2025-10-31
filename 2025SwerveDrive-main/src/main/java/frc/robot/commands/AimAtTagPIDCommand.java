package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionAim;
import frc.robot.subsystems.VisionAim.PathPlan;
import frc.robot.Constants.VisionConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

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

        PathPlan plan = vision.getPlan(robot2goal);
        if (plan == null) {
            System.out.println("AimAtTagPID: No valid tag plan found.");
            return;
        }

        // Compute goal in *field space* (odometry + vision offset)
        Pose2d robotPose = drivetrain.getPose();
        goalPose = robotPose.transformBy(plan.robotToGoal);
        hasTarget = true;

        System.out.println("[AimAtTagPID] Moving toward tag " + plan.tagId +
                " target at (" + String.format("%.3f", goalPose.getX()) + ", " +
                String.format("%.3f", goalPose.getY()) + ")");
    }

    @Override
    public void execute() {
        if (!hasTarget) return;

        // Get current position
        Pose2d currentPose = drivetrain.getPose();
        System.out.println("POSE WHILE AIMING: " + currentPose);

        // Compute error in *field coordinates*
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        // Rotate that error into robot-relative frame
        Translation2d fieldError = new Translation2d(dx, dy);
        Translation2d robotError = fieldError.rotateBy(currentPose.getRotation().unaryMinus());

        double xErr = robotError.getX();  // forward/backward
        double yErr = robotError.getY();  // left/right

        // Apply deadbands
        if (Math.abs(xErr) < VisionConstants.XY_DEADBAND_M) xErr = 0.0;
        if (Math.abs(yErr) < VisionConstants.XY_DEADBAND_M) yErr = 0.0;

        // Proportional control
        double vx = VisionConstants.KP_XY * xErr;
        double vy = VisionConstants.KP_XY * yErr;
        double omega = 0.0; // still no rotation

        // Clamp speeds
        vx = MathUtil.clamp(vx, -VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);
        vy = MathUtil.clamp(vy, -VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);

        drivetrain.setOperatorPerspectiveForward(Rotation2d.kZero);
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(vx, vy, omega))
                .withCenterOfRotation(new Translation2d(0, 0))
        );
    }

    @Override
    public boolean isFinished() {
        if (!hasTarget) return true;

        Pose2d currentPose = drivetrain.getPose();
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();
        double dist = Math.hypot(dx, dy);

        return dist < VisionConstants.GOAL_POS_EPS_M;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("[AimAtTagPID] " + (interrupted ? "Interrupted" : "Completed") + ".");
    }
}