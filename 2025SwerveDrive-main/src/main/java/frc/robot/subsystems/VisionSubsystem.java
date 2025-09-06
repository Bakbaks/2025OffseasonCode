package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera rightCam = new PhotonCamera(VisionConstants.RIGHT_CAM_NAME);
  private final PhotonCamera leftCam  = new PhotonCamera(VisionConstants.LEFT_CAM_NAME);

  // Robot to Camera extrinsics
  private final Transform3d robotToRight3d = new Transform3d(
      new Translation3d(VisionConstants.R_X, VisionConstants.R_Y, VisionConstants.R_Z),
      new Rotation3d(VisionConstants.R_ROLL, VisionConstants.R_PITCH, VisionConstants.R_YAW));

  private final Transform3d robotToLeft3d = new Transform3d(
      new Translation3d(VisionConstants.L_X, VisionConstants.L_Y, VisionConstants.L_Z),
      new Rotation3d(VisionConstants.L_ROLL, VisionConstants.L_PITCH, VisionConstants.L_YAW));

  // images stored
  private volatile PhotonPipelineResult rRes = new PhotonPipelineResult();
  private volatile PhotonPipelineResult lRes = new PhotonPipelineResult();

  @Override
  public void periodic() {
    rRes = rightCam.getLatestResult();
    lRes = leftCam.getLatestResult();
  }

  /** Best single Robot to Goal transform, relative to the robot (2D). */
  public Optional<Transform2d> getBestRobotToGoal() {
    var candidates = new ArrayList<Candidate>();
    candidates.addAll(buildCandidates(rRes, robotToRight3d, VisionConstants.RIGHT_CAM_NAME));
    candidates.addAll(buildCandidates(lRes,  robotToLeft3d,  VisionConstants.LEFT_CAM_NAME));

    System.out.println("comparing left and right goals - right goal: " + robotToRight3d + " left goal: " + robotToLeft3d);
    

    return candidates.stream()
        .min(Comparator
              .comparingDouble((Candidate c) -> c.ambiguity)
              .thenComparingDouble(c -> c.dist))
        .map(c -> c.rToGoal);


  }

  /** All viable Robot to Goal transforms (sorted from best to worst). */
  public List<Transform2d> getAllRobotToGoals() {
    var candidates = new ArrayList<Candidate>();
    candidates.addAll(buildCandidates(rRes, robotToRight3d, VisionConstants.RIGHT_CAM_NAME));
    candidates.addAll(buildCandidates(lRes,  robotToLeft3d,  VisionConstants.LEFT_CAM_NAME));

    candidates.sort(Comparator
        .comparingDouble((Candidate c) -> c.ambiguity)
        .thenComparingDouble(c -> c.dist));

    var out = new ArrayList<Transform2d>();
    for (var c : candidates) out.add(c.rToGoal);
    return out;
  }


  private static final class Candidate {
    final Transform2d rToGoal;  // Robot to Goal (2D)
    final double ambiguity;     // smaller is better (0..1);
    final double dist;          // camera to tag distance for tie-breakers
    final String camName;
    final int tagId;

    Candidate(Transform2d rToGoal, double ambiguity, double dist, String camName, int tagId) {
      this.rToGoal = rToGoal;
      this.ambiguity = ambiguity;
      this.dist = dist;
      this.camName = camName;
      this.tagId = tagId;
    }

  }

  private List<Candidate> buildCandidates(PhotonPipelineResult res, Transform3d robotToCam3d, String camName) {
    var list = new ArrayList<Candidate>();
    if (res == null || !res.hasTargets()) {
      System.out.println((res == null) ? "CAMERA OFFLINE--------------------" : "CAMERA HAS NO TARGETS--------------------");
      return list;
    }

    // Convert 3d to planar Transform2d
    Transform2d rToCam2d = transform3dTo2d(robotToCam3d);

    
    for (PhotonTrackedTarget t : res.getTargets()) {
      int id = t.getFiducialId();
      if (id <= 0) continue;

      Transform3d camToTag3d = t.getBestCameraToTarget();

      Transform2d cToTag2d;
      double dist, amb = t.getPoseAmbiguity();
      if (camToTag3d != null) {
        // Use full Perspective n point, then project to 2D
        cToTag2d = transform3dTo2d(camToTag3d);
        dist = camToTag3d.getTranslation().getNorm();
        if (amb < 0) amb = 1.0;
      } else {
        // estimate distance from pitch + heights: build a planar cam to tag from yaw.
        double camH   = robotToCam3d.getTranslation().getZ();
        double tagH   = VisionConstants.TAG_HEIGHT_M;
        double camPit = robotToCam3d.getRotation().getY(); // pitch (rad)
        double yawCam = Math.toRadians(t.getYaw());

        dist = PhotonUtils.calculateDistanceToTargetMeters(camH, tagH, camPit, Math.toRadians(t.getPitch()));
        if (!Double.isFinite(dist) || dist <= 0) continue;

        Translation2d planar = new Translation2d(dist * Math.cos(yawCam), dist * Math.sin(yawCam));
        cToTag2d = new Transform2d(planar, new Rotation2d(yawCam));
        amb = 1.0;
      }

      Transform2d rToGoal = rToCam2d.plus(cToTag2d).plus(VisionConstants.TAG_TO_GOAL);

  
      if (amb > 0.01) continue;        // dont choose highly ambiguous
      if (Math.abs(dist) > 0) continue;        // dont choose far-away reads

      list.add(new Candidate(rToGoal, amb, dist, camName, id));
    }
    
    return list;
    }

  // Project a 3D transform into the robot’s drive plane (X,Y + yaw only).
  private static Transform2d transform3dTo2d(Transform3d t3) {
    Translation3d tr = t3.getTranslation();
    Rotation3d r3 = t3.getRotation();
    return new Transform2d(
        new Translation2d(tr.getX(), tr.getY()),
        r3.toRotation2d()  // yaw as Rotation2d
    );
  }
}