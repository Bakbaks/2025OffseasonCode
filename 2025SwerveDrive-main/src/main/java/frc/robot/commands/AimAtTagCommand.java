package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;
import java.util.Optional;

public class AimAtTagCommand extends Command {

  //path planning
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionSubsystem vision;
  private final PathConstraints limits;
  private final double minPlanDistanceM;

  private Command followCmd;
  private Pose2d targetPose;
  private Transform2d robot2goal;

  
  public AimAtTagCommand(
    Transform2d robot2goal,
    CommandSwerveDrivetrain drivetrain,
    VisionSubsystem vision,
    PathConstraints limits,
    double minPlanDistanceM) {
    this.robot2goal = robot2goal;
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.limits = limits;
    this.minPlanDistanceM = minPlanDistanceM;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    followCmd = null;
    
    
    VisionSubsystem.PathPlan plan = vision.getPlan(robot2goal);
    if (plan == null) {
      System.out.println("No vision plan");
      return;
    }else{
      System.out.println("yes vision plan");
    }

    System.out.println("--------------------" + plan + "----------------------");
    
    Pose2d start = drivetrain.getPose();
    Transform2d robotToGoal = plan.robotToGoal;
    System.out.println("---------------------------------GOALLLLL" + robotToGoal);
    Pose2d end = start.transformBy(robotToGoal);

    Rotation2d finalHeading = end.getRotation();
    targetPose = new Pose2d(end.getTranslation(), finalHeading);

    double dx = targetPose.getX() - start.getX();
    double dy = targetPose.getY() - start.getY();
    double dist = Math.hypot(dx, dy);
    if (dist < minPlanDistanceM) {
      System.out.printf("close enouugh distance: ", dist);
      return;
    }

    
    //path planner magic with waypoints
    Rotation2d travelDir = new Rotation2d(dx, dy);
    Pose2d startForPath = new Pose2d(start.getTranslation(), travelDir);
    Pose2d endForPath   = new Pose2d(targetPose.getTranslation(), travelDir);
    List<Waypoint> wps = PathPlannerPath.waypointsFromPoses(startForPath, endForPath);

    
    GoalEndState goal = new GoalEndState(0.0, finalHeading);

    
    PathPlannerPath path = new PathPlannerPath(wps, limits, null, goal, false);

    followCmd = AutoBuilder.followPath(path);
    System.out.println("Following path to tag " + plan.tagId + " via " + plan.cameraName);
    followCmd.initialize();

  
  }

  @Override
  public void execute() {
    if (followCmd != null) {
      followCmd.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return followCmd == null || followCmd.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (followCmd != null) followCmd.end(interrupted);
    drivetrain.stop();
  }
}