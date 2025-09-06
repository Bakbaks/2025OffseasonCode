package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

public class AimAtTagCommand extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final VisionSubsystem vision;

  private final SwerveRequest.RobotCentric driveReq = new SwerveRequest.RobotCentric();

  public AimAtTagCommand(CommandSwerveDrivetrain swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  private Optional<Transform2d> lastGoal = Optional.empty();

  @Override
  public void initialize() {
    lastGoal = Optional.empty();
  }

  @Override
  public void execute() {
    Optional<Transform2d> maybeR2G = vision.getBestRobotToGoal();
    if (maybeR2G.isPresent()) lastGoal = maybeR2G;

    if (lastGoal.isEmpty()) {
      swerve.setControl(driveReq.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }

    Transform2d r2g = lastGoal.get();

    double dx = r2g.getX();                          // +X forward
    double dy = r2g.getY();                          // +Y left
    double dtheta = r2g.getRotation().getRadians();  // +CCW

    // Deadbands
    if (Math.abs(dx) < VisionConstants.XY_DEADBAND_M) dx = 0.0;
    if (Math.abs(dy) < VisionConstants.XY_DEADBAND_M) dy = 0.0;
    if (Math.abs(dtheta) < VisionConstants.TH_DEADBAND_RAD) dtheta = 0.0;

    // Proportional control to desired speeds
    double vx = MathUtil.clamp(VisionConstants.KP_XY * dx,-VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);
    double vy = MathUtil.clamp(VisionConstants.KP_XY * dy,-VisionConstants.MAX_VX_M_PER_S, VisionConstants.MAX_VX_M_PER_S);
    double omega = MathUtil.clamp(VisionConstants.KP_THETA * dtheta,-VisionConstants.MAX_OMEGA_RAD_PER_S, VisionConstants.MAX_OMEGA_RAD_PER_S);

    System.out.println("sending commands to SWERVE DRIVE vx=" + vx + " vy=" + vy + " omega=" + omega);
    swerve.setControl(
        driveReq.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(driveReq.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    if (lastGoal.isEmpty()) return false;
    var g = lastGoal.get();
    Translation2d t = g.getTranslation();
    boolean posDone = t.getNorm() < VisionConstants.GOAL_POS_EPS_M;
    boolean angDone = Math.abs(g.getRotation().getRadians()) < VisionConstants.GOAL_ANG_EPS_RAD;
    return posDone && angDone;
  }
}