package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.PathPlan;
import frc.robot.Constants.VisionConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimAtTagPIDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Transform2d robot2goal;

    private Transform2d currentError;
    private boolean hasTarget = false;

    public AimAtTagPIDCommand(Transform2d robot2goal, CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
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

        currentError = plan.robotToGoal;
        hasTarget = true;

        System.out.println("[AimAtTagPID] Moving toward tag " + plan.tagId + 
                           " with offset (x=" + String.format("%.3f", currentError.getX()) + 
                           " y=" + String.format("%.3f", currentError.getY()) + ")");
    }

    @Override
    public void execute() {
        if (!hasTarget) return;

        
        double xErr = currentError.getX(); // forward/backward
        double yErr = currentError.getY(); // left/right

        
        if (Math.abs(xErr) < VisionConstants.XY_DEADBAND_M) xErr = 0;
        if (Math.abs(yErr) < VisionConstants.XY_DEADBAND_M) yErr = 0;

        
        double vx = VisionConstants.KP_XY * xErr; // forward speed
        double vy = VisionConstants.KP_XY * yErr; // strafe speed
        double omega = 0.0;

       
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

        double xErr = Math.abs(currentError.getX());
        double yErr = Math.abs(currentError.getY());

        // Only finish once the robot is close enough in position
        return xErr < VisionConstants.GOAL_POS_EPS_M &&
               yErr < VisionConstants.GOAL_POS_EPS_M;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        System.out.println("[AimAtTagPID] " + (interrupted ? "Interrupted" : "Completed") + ".");
    }
}
