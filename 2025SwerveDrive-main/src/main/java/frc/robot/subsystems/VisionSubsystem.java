package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  // Cameras
  private final PhotonCamera rightCamera = new PhotonCamera(VisionConstants.RIGHT_CAM_NAME);
  private final PhotonCamera leftCamera  = new PhotonCamera(VisionConstants.LEFT_CAM_NAME);

  //camera extrinsics
  private final Transform3d robotToRight = new Transform3d(
      new Translation3d(VisionConstants.R_X, VisionConstants.R_Y, VisionConstants.R_Z),
      new Rotation3d(VisionConstants.R_ROLL, VisionConstants.R_PITCH, VisionConstants.R_YAW));

  private final Transform3d robotToLeft = new Transform3d(
      new Translation3d(VisionConstants.L_X, VisionConstants.L_Y, VisionConstants.L_Z),
      new Rotation3d(VisionConstants.L_ROLL, VisionConstants.L_PITCH, VisionConstants.L_YAW));

  public static class Snapshot {
    public long timestampMs;
    public List<Tag> observed = new ArrayList<>();
  }

  public static class Tag {
    public int tagId;
    public Transform2d camToTag2d;
    public double ambiguity;
  }

  public static class PathPlan {
    public String cameraName;
    public int tagId;
    public Transform2d robotToGoal;

    @Override
    public String toString() {
      return "[Plan cam=" + cameraName + " tag=" + tagId +
             " dx=" + String.format("%.3f", robotToGoal.getX()) +
             " dy=" + String.format("%.3f", robotToGoal.getY()) +
             " dtheta=" + String.format("%.3f", robotToGoal.getRotation().getDegrees()) + " DEG]";
    }
  }

  
  public PathPlan getPlan(Transform2d robot2goal) {
    Snapshot rightSnap = takeSnapshot(rightCamera, robotToRight);
    Snapshot leftSnap  = takeSnapshot(leftCamera,  robotToLeft);

    Tag rightFirst = getTag(rightSnap);
    Tag leftFirst  = getTag(leftSnap);

    if (rightFirst == null && leftFirst == null) {
      System.out.println("[Vision] No tags on either camera.");
      return null;
    }

    //choose lower photon vision ambiguity
    boolean chooseRight = false;
    if (rightFirst != null && leftFirst == null) {
      chooseRight = true;
    } else if (rightFirst == null && leftFirst != null) {
      chooseRight = false;
    } else {
      chooseRight = rightFirst.ambiguity <= leftFirst.ambiguity;
    }

    // Robot to Goal = Robot to Cam   +   Cam to Tag    + Tag to Goal
    PathPlan plan = new PathPlan();
    if (chooseRight) {
      plan.cameraName = "right";
      plan.tagId = rightFirst.tagId;
      Transform2d robotToCam2d = project3d2d(robotToRight);
      plan.robotToGoal = robotToCam2d.plus(rightFirst.camToTag2d).plus(robot2goal);
    } else {
      plan.cameraName = "left";
      plan.tagId = leftFirst.tagId;
      Transform2d robotToCam2d = project3d2d(robotToLeft);
      plan.robotToGoal = robotToCam2d.plus(leftFirst.camToTag2d).plus(robot2goal);
    }

    System.out.println("[Vision] " + plan);
    return plan;
  }


  private Snapshot takeSnapshot(PhotonCamera cam, Transform3d robotToCam) {
    Snapshot snap = new Snapshot();
    snap.timestampMs = System.currentTimeMillis();

    PhotonPipelineResult result = cam.getLatestResult();// change to undeprecated method
    if (result == null || !result.hasTargets()) {
      return snap;
    }

    for (PhotonTrackedTarget t : result.getTargets()) {
      if (t == null) continue;
      if (t.getFiducialId() <= 0 
      || t.getFiducialId() == 13
      || t.getFiducialId() == 12
      || t.getFiducialId() == 16
      || t.getFiducialId() == 14
      || t.getFiducialId() == 15
      || t.getFiducialId() == 4
      || t.getFiducialId() == 5
      || t.getFiducialId() == 3
      || t.getFiducialId() == 2
      || t.getFiducialId() == 1) continue; // photon vision function that checks if the april tag is legit

      Tag obs = new Tag();
      obs.tagId = t.getFiducialId();
      obs.ambiguity = t.getPoseAmbiguity();
      if (obs.ambiguity < 0) obs.ambiguity = 1.0; // unknown ambiguities are set to the highest

      // use photon vision 3d pose only if available
      Transform3d camToTag3d = t.getBestCameraToTarget();
      if (camToTag3d != null) {
        System.out.println("----------------" + camToTag3d + "----------____________");
        obs.camToTag2d = project3d2d(camToTag3d);
      } else {
        double camHeight = robotToCam.getTranslation().getZ();
        double tagHeight = VisionConstants.TAG_HEIGHT_M;
        double camPitch  = robotToCam.getRotation().getY();

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            camHeight, tagHeight, camPitch, Math.toRadians(t.getPitch()));

        if (!Double.isFinite(distance) || distance <= 0) {
          System.out.println("No Photon vision Pose and Detected an invalid target");
          continue; // invavlid results
          
        }

        double yawRad = Math.toRadians(t.getYaw());
        Translation2d planar = new Translation2d(distance * Math.cos(yawRad),
                                                 distance * Math.sin(yawRad));
        obs.camToTag2d = new Transform2d(planar, new Rotation2d(yawRad));
      }

      snap.observed.add(obs);
    }

    return snap;
  }

  private Tag getTag(Snapshot snap) {
    if (snap == null || snap.observed == null || snap.observed.isEmpty()) return null;
  
    Tag best = null;
    double bestDist = Double.POSITIVE_INFINITY;
  
    for (Tag t : snap.observed) {
      if (t.camToTag2d == null) continue;
      double dx = t.camToTag2d.getX();
      double dy = t.camToTag2d.getY();
      double planar = Math.hypot(dx, dy);   // distance in the X–Y plane
  
      // prefer lower ambiguity when distances are ~equal
      if (planar < bestDist - 1e-6 ||
          (Math.abs(planar - bestDist) <= 1e-6 && best != null && t.ambiguity < best.ambiguity)) {
        best = t;
        bestDist = planar;
      }
    }
    return best;
  }

  //x,y,yaw
  private static Transform2d project3d2d(Transform3d t3) {
    Translation3d tr = t3.getTranslation();
    Rotation3d r3 = t3.getRotation();

    Rotation2d yaw2d = t3.getRotation().toRotation2d();
    
    double deg = yaw2d.getDegrees();

    deg = Math.IEEEremainder(deg, 360.0);
    
    if (deg > 90.0) {
      deg = -(180.0 - deg);
    } else if (deg < -90.0) {
      deg = 180.0 + deg;  
    }

    Rotation2d folded = Rotation2d.fromDegrees(deg);

    return new Transform2d(new Translation2d(tr.getX(), tr.getY()), folded);
  }
}