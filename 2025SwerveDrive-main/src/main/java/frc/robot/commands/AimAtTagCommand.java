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
  
  // Tracking state
  private double lastValidTargetDistance = Double.POSITIVE_INFINITY;
  private static final double CLOSE_DISTANCE_THRESHOLD = 1.0; // meters
  private static final double SIGNIFICANT_POSE_CHANGE = 0.1; // meters
  private int debugCounter = 0;
  
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
    debugCounter++;
    Pose2d currentPose = drivetrain.getPose();
    
    // Try to update vision if we're not too close
    if (lastValidTargetDistance > CLOSE_DISTANCE_THRESHOLD || targetPose == null) {
      VisionSubsystem.PathPlan plan = vision.getPlan(robot2goal);
      if (plan != null) {
        Transform2d robotToGoal = plan.robotToGoal;
        Pose2d newTargetPose = currentPose.transformBy(robotToGoal);
        
        // Calculate distance to target
        double dx = newTargetPose.getX() - currentPose.getX();
        double dy = newTargetPose.getY() - currentPose.getY();
        lastValidTargetDistance = Math.hypot(dx, dy);
        
        boolean significantChange = 
            targetPose == null || 
            Math.abs(targetPose.getX() - newTargetPose.getX()) > SIGNIFICANT_POSE_CHANGE ||
            Math.abs(targetPose.getY() - newTargetPose.getY()) > SIGNIFICANT_POSE_CHANGE;
            
        // Update path if target moved significantly
        if (significantChange) {
          targetPose = newTargetPose;
          updatePath(currentPose, dx, dy, plan.tagId);
        }
        
        // Debug output every 10 cycles
        if (debugCounter % 10 == 0) {
          System.out.printf("[AutoAim] Dist:%.2fm Tag:%d Vision:%s Mem:%b dX:%.2f dY:%.2f\n",
              lastValidTargetDistance, 
              plan.tagId,
              plan.cameraName,
              lastValidTargetDistance <= CLOSE_DISTANCE_THRESHOLD,
              dx, dy);
        }
      }
    }
    
    // Execute current path
    if (followCmd != null) {
      followCmd.execute();
    }
  }
  
  private void updatePath(Pose2d start, double dx, double dy, int tagId) {
    Rotation2d travelDir = new Rotation2d(dx, dy);
    Pose2d startForPath = new Pose2d(start.getTranslation(), travelDir);
    Pose2d endForPath = new Pose2d(targetPose.getTranslation(), travelDir);
    
    List<Waypoint> wps = PathPlannerPath.waypointsFromPoses(startForPath, endForPath);
    GoalEndState goal = new GoalEndState(0.0, targetPose.getRotation());
    PathPlannerPath path = new PathPlannerPath(wps, limits, null, goal, false);
    
    // Switch to new path
    if (followCmd != null) {
      followCmd.end(false);
    }
    followCmd = AutoBuilder.followPath(path);
    followCmd.initialize();
    
    System.out.println("[AutoAim] New path to tag " + tagId + 
                      String.format(" dist=%.2fm heading=%.1f°", 
                      lastValidTargetDistance,
                      targetPose.getRotation().getDegrees()));
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