package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import java.util.Optional;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {

 List<Integer> reefIDs =
      new ArrayList<Integer>(Arrays.asList(19, 20, 21, 22, 17, 18, 6, 7, 8, 9, 10, 11));
  List<Integer> blueReefID = new ArrayList<Integer>(Arrays.asList(19, 20, 21, 22, 17, 18));
  List<Integer> redReefID = new ArrayList<Integer>(Arrays.asList(6, 7, 8, 9, 10, 11));

  private Pose2d savedResult = new Pose2d(0, 0, Rotation2d.fromRadians(0));
  private static VisionSubsystem[] systemList =
      new VisionSubsystem[Constants.VisionConstants.Cameras.values().length];
  private Transform3d[] camToRobots = {
    // right Camera transform
    new Transform3d(
        new Translation3d(
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_TRANSLATION_X,
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_TRANSLATION_Y,
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_TRANSLATION_Z),
        new Rotation3d(
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_ROTATION_ROLL,
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_ROTATION_PITCH,
            Constants.VisionConstants.RIGHT_CAM_TO_ROBOT_ROTATION_YAW)),
    // left Camera
    new Transform3d(
        new Translation3d(
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_TRANSLATION_X,
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_TRANSLATION_Y,
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_TRANSLATION_Z),
        new Rotation3d(
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_ROTATION_ROLL,
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_ROTATION_PITCH,
            Constants.VisionConstants.LEFT_CAM_TO_ROBOT_ROTATION_YAW)),

    
  };

  private PhotonCamera camera;
  private Constants.VisionConstants.Cameras cameraEnum;
  private PhotonPipelineResult pipeline;

  private static final Path fieldJsonPath =
      Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape-andymark.json");
  private static AprilTagFieldLayout aprilTagFieldLayout;

  static {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(fieldJsonPath);
      System.out.println("Loaded PhtonVision field layout successfully.");
    } catch (Exception e) {
      System.err.println("Failed to load AprilTagFieldLayout: " + e.getMessage());
    }
 }

  
 
  PhotonPoseEstimator photonPoseEstimator;
  private CommandSwerveDrivetrain drivetrain;
  private BooleanSupplier redSide;

  public VisionSubsystem(Constants.VisionConstants.Cameras cameraEnum, BooleanSupplier redSide, CommandSwerveDrivetrain drivetrain) {
    this.cameraEnum = cameraEnum;
    String name = cameraEnum.toString();
    int index = cameraEnum.ordinal();
    camera = new PhotonCamera(name);
     
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobots[index]);
    this.redSide = redSide;
    this.drivetrain = drivetrain;

    try {
        if (redSide.getAsBoolean()) {
            aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            System.out.println("[VisionSubsystem] Set field origin to RED alliance side.");
        } else {
            aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            System.out.println("[VisionSubsystem] Set field origin to BLUE alliance side.");
        }
    } catch (Exception e) {
        System.err.println("[VisionSubsystem] Failed to set field origin: " + e.getMessage());
    }

    
    for (var x : camera.getAllUnreadResults()) {
      pipeline = x;
    }
  }
  
  
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return this.aprilTagFieldLayout;
  }

  public PhotonPipelineResult getPipelineResult() {
    return pipeline;
  }

  public Optional<EstimatedRobotPose> getMultiTagPose3d(Pose2d previousRobotPose) {
    if (pipeline == null) return Optional.empty();       
    photonPoseEstimator.setReferencePose(previousRobotPose);
    return photonPoseEstimator.update(pipeline);
  }

   public Pose2d getPose2d() {
    Optional<EstimatedRobotPose> pose3d = getMultiTagPose3d(savedResult);
    if (pose3d.isEmpty()) return savedResult;
    savedResult = pose3d.get().estimatedPose.toPose2d();
    return savedResult;
  }

  public Pose2d getSaved() {
    return savedResult;
  }

  
  public Double getDistance() {
    return this.getAprilTagFieldLayout()
        .getTagPose(getPipelineResult().getBestTarget().getFiducialId())
        .get()
        .getTranslation()
        .getDistance(
            new Translation3d(
                drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), 0.0));
  }

  public static VisionSubsystem getInstance(Constants.VisionConstants.Cameras name, BooleanSupplier redSide, CommandSwerveDrivetrain drivetrain) {
    if (systemList[name.ordinal()] == null) {
      systemList[name.ordinal()] = new VisionSubsystem(name, redSide, drivetrain);
    }

    return systemList[name.ordinal()];
  }
   

  public boolean hasTarget(PhotonPipelineResult pipeline) {
    if (pipeline == null) {
      return false;
    }
    return pipeline.hasTargets();
  }


  public void setReference(Pose2d newPose) {
    if (newPose == null) {
      return;
    }
    savedResult = newPose;
  }

  public record VisionEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public Optional<VisionEstimate> getEstimatedPose() {
    PhotonPipelineResult pipelineResult = getPipelineResult();

    // Basic validity checks
    if (pipelineResult == null || !hasTarget(pipelineResult)) return Optional.empty();

    // Compute the estimated pose
    Optional<EstimatedRobotPose> estimated3d = getMultiTagPose3d(savedResult);
    if (estimated3d.isEmpty()) return Optional.empty();

    Pose2d estimatedPose = estimated3d.get().estimatedPose.toPose2d();
    savedResult = estimatedPose; // keep track

    // Calculate a default vision uncertainty matrix (can be tuned per camera)
    double distance = 1.0;
    try {
        int tagID = pipelineResult.getBestTarget().getFiducialId();
        distance = this.getAprilTagFieldLayout()
            .getTagPose(tagID)
            .get()
            .getTranslation()
            .getDistance(new Translation3d(estimated3d.get().estimatedPose.getTranslation().getX(),
                                           estimated3d.get().estimatedPose.getTranslation().getY(),
                                           0));
    } catch (Exception e) {
        // fallback if tag info not available
    }

    // Decrease trust as distance increases
    double xStd = 0.1 + 0.2 * distance;
    double yStd = 0.1 + 0.2 * distance;
    double thetaStd = Math.toRadians(10); // ~10 degree uncertainty
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xStd, yStd, thetaStd);

    return Optional.of(new VisionEstimate(estimatedPose, pipelineResult.getTimestampSeconds(), stdDevs));
}



  @Override
  public void periodic() {
    for (var x : camera.getAllUnreadResults()) {
      pipeline = x;
    }
  }

  
}