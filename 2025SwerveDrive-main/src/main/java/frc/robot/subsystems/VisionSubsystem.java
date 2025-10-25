package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.MiscUtils;

public class VisionSubsystem extends SubsystemBase {
  public static int numThrowaways = 0;
  public static int numNotThrowaways = 0;
  
  // AprilTag IDs for field elements
  private List<Integer> reefIDs =
      new ArrayList<Integer>(Arrays.asList(19, 20, 21, 22, 17, 18, 6, 7, 8, 9, 10, 11));
  private List<Integer> blueReefID = new ArrayList<Integer>(Arrays.asList(19, 20, 21, 22, 17, 18));
  private List<Integer> redReefID = new ArrayList<Integer>(Arrays.asList(6, 7, 8, 9, 10, 11));

  // Field layout and pose estimation
  private AprilTagFieldLayout aprilTagFieldLayout = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private PhotonPoseEstimator rightPoseEstimator;
  private PhotonPoseEstimator leftPoseEstimator;
  private Pose2d savedResult = new Pose2d(0, 0, new Rotation2d(0.01, 0.01));
  private BooleanSupplier redSide;
  private SwerveSubsystem driveTrain;

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

  public VisionSubsystem(BooleanSupplier redSide) {
    this.redSide = redSide;
    this.driveTrain = SwerveSubsystem.getInstance();

    // Initialize pose estimators for both cameras
    rightPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightCamera,
        robotToRight);

    leftPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftCamera,
        robotToLeft);
  }

  public PathPlan getPlan(Transform2d robot2goal) {
    // Take snapshots from both cameras
    Snapshot rightSnap = takeSnapshot(rightCamera, robotToRight);
    Snapshot leftSnap = takeSnapshot(leftCamera, robotToLeft);

    // Get best tag from each camera
    Tag rightFirst = getTag(rightSnap);
    Tag leftFirst = getTag(leftSnap);

    if (rightFirst == null && leftFirst == null) {
      DogLog.log("Vision/NullTags", true);
      return null;
    }

    // Choose camera with lower ambiguity
    boolean chooseRight = false;
    if (rightFirst != null && leftFirst == null) {
      chooseRight = true;
    } else if (rightFirst == null && leftFirst != null) {
      chooseRight = false;
    } else {
      chooseRight = rightFirst.ambiguity <= leftFirst.ambiguity;
    }

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

    DogLog.log("Vision/Plan", plan.toString());
    return plan;
  }

  @Override
  public void periodic() {
    // Process vision data from both cameras
    addFilteredPose(rightCamera, rightPoseEstimator, "RightCamera");
    addFilteredPose(leftCamera, leftPoseEstimator, "LeftCamera");
  }

    // Initialize pose estimators for both cameras
    rightPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightCamera,
        robotToRight);

    leftPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftCamera,
        robotToLeft);
  }

  public void addFilteredPose(PhotonCamera camera, PhotonPoseEstimator poseEstimator, String camName) {
    PhotonPipelineResult pipelineResult = camera.getLatestResult();

    DogLog.log("KalmanDebug/" + camName + "PiplineNull", pipelineResult == null);
    DogLog.log("KalmanDebug/" + camName + "PipelineHasTarget", pipelineResult.hasTargets());

    if (pipelineResult != null && pipelineResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
      boolean hasReefTag = true;
      double poseAmbiguity = pipelineResult.getBestTarget().getPoseAmbiguity();

      // Check if we see valid reef tags for our alliance
      for (PhotonTrackedTarget target : targets) {
        if (redSide.getAsBoolean()) {
          if (!redReefID.contains(target.getFiducialId())) {
            hasReefTag = false;
          }
        } else {
          if (!blueReefID.contains(target.getFiducialId())) {
            hasReefTag = false;
          }
        }
      }

      if (hasReefTag) {
        // Get estimated pose
        Optional<EstimatedRobotPose> result = poseEstimator.update(pipelineResult);
        if (result.isEmpty()) return;

        EstimatedRobotPose pose = result.get();
        Pose2d robotPose = pose.estimatedPose.toPose2d();

        // Calculate Kalman filter values based on robot speed and distance
        double speed = Math.hypot(
            driveTrain.getRobotSpeeds().vxMetersPerSecond,
            driveTrain.getRobotSpeeds().vyMetersPerSecond);
        double speedMultiplier = (speed / 2.0) + 1.0;

        double bestTagDistance = pipelineResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        double xKalman = MiscUtils.lerp((bestTagDistance - 0.62) / 3, 0.03, 0.3, 1.0) * speedMultiplier;
        double yKalman = MiscUtils.lerp((bestTagDistance - 0.62) / 3, 0.03, 0.3, 1.0) * speedMultiplier;
        double rotationKalman = MiscUtils.lerp((bestTagDistance - 0.6) / 1.4, 0.4, 5, 30) / 10;

        // Create vision measurement matrix
        Matrix<N3, N1> visionMatrix = VecBuilder.fill(xKalman, yKalman, rotationKalman);
        
        // Add vision measurement to drivetrain pose estimator
        double timestamp = pipelineResult.getTimestampSeconds();
        double timeDiff = Math.abs(timestamp - Timer.getFPGATimestamp());
        
        if (timeDiff > 5) {
          timestamp = Timer.getFPGATimestamp() - 0.070; // Use recent timestamp if too old
        }

        driveTrain.addVisionMeasurement(robotPose, timestamp, visionMatrix);
        
        // Log debug info
        DogLog.log("KalmanDebug/" + camName + "PoseAmbiguity", poseAmbiguity);
        DogLog.log("KalmanDebug/" + camName + "DistToAprilTag", bestTagDistance);
        DogLog.log("KalmanDebug/" + camName + "TranslationStdDev", xKalman);
        DogLog.log("KalmanDebug/" + camName + "RotationStdDev", rotationKalman);
        DogLog.log("KalmanDebug/" + camName + "VisionPose", robotPose);
        DogLog.log("KalmanDebug/" + camName + "TimeDiff", timeDiff);
        DogLog.log("KalmanDebug/" + camName + "visionUsed", true);
      }
    }
  }

  //x,y,yaw
  private static Transform2d project3d2d(Transform3d t3) {
    Translation3d tr = t3.getTranslation();
    Rotation3d r3 = t3.getRotation();

    Rotation2d yaw2d = t3.getRotation().toRotation2d();
    double origDeg = yaw2d.getDegrees();

    double deg = Math.IEEEremainder(origDeg, 360.0);

    boolean flipped = false;
    if (deg > 90.0) {
      // fold across the 180 boundary and note we flipped the facing 180°
      deg = -(180.0 - deg);
      flipped = true;
    } else if (deg < -90.0) {
      deg = 180.0 + deg;
      flipped = true;
    }

    // If we folded the yaw by 180°, the translation in the camera frame should be
    // rotated by 180° as well (equivalent to negating X and Y) to keep the
    // pose consistent. Failing to do this can leave translation and rotation
    // inconsistent and cause path/heading to be flipped (robot driving "backwards").
    double tx = tr.getX();
    double ty = tr.getY();
    if (flipped) {
      tx = -tx;
      ty = -ty;
    }

    Rotation2d folded = Rotation2d.fromDegrees(deg);

    return new Transform2d(new Translation2d(tx, ty), folded);
  }
}