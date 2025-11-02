package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;

import java.util.Comparator;
import java.util.Optional;
import java.util.Set;

/**
 * VisionAim (odometry-only version)
 * Uses robot odometry and AprilTagFieldLayout to find the nearest tag
 * and compute a goal pose offset from it.
 */
public class VisionAim extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;
    private final CommandSwerveDrivetrain drivetrain;

    // Only include tags from your alliance or relevant field side
    private static final Set<Integer> ALLOWED_TAG_IDS =
            Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    public VisionAim(AprilTagFieldLayout sharedLayout, CommandSwerveDrivetrain drivetrain) {
        this.fieldLayout = sharedLayout;
        this.drivetrain = drivetrain;

        if (sharedLayout != null)
            System.out.println("[VisionAim] Using shared AprilTagFieldLayout (odometry-only mode).");
        else
            System.err.println("[VisionAim] WARNING: Field layout is null!");
    }

    /**
     * Finds the nearest tag to the robot based purely on odometry.
     *
     * @param robotPose current robot pose from odometry
     * @return nearest tag's field-space pose
     */
    public Optional<Pose3d> getNearestTagPose(Pose2d robotPose) {
        if (fieldLayout == null || fieldLayout.getTags().isEmpty()) return Optional.empty();

        return fieldLayout.getTags().stream()
                .filter(tag -> ALLOWED_TAG_IDS.contains(tag.ID))
                .map(tag -> tag.pose)
                .min(Comparator.comparingDouble(
                        pose -> pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation())));
    }

    /**
     * Computes a goal pose offset from the nearest tag, factoring in tag yaw (rotation).
     * The goal will face the tag (rotated 180° from tag yaw).
     *
     * @param robotPose current robot pose
     * @param tagOffset desired offset in the tag's coordinate frame
     *                  (e.g. right: new Translation2d(0, -0.5), front: new Translation2d(-0.5, 0))
     * @return field-space goal pose
     */
    public Optional<Pose2d> computeNearestGoalPose(Pose2d robotPose, Transform2d tagOffset) {
        Optional<Pose3d> nearestTag = getNearestTagPose(robotPose);
        if (nearestTag.isEmpty()) return Optional.empty();

        Pose2d tagPose2d = nearestTag.get().toPose2d();

        // Apply the offset in the tag's coordinate frame (factoring in its yaw)
        Pose2d offsetPose = tagPose2d.transformBy(tagOffset);

        // Rotate goal so it faces the tag (180 degrees opposite tag's yaw)
        Rotation2d facingTag = tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180));
        Pose2d goalPose = new Pose2d(offsetPose.getTranslation(), facingTag);

        // 🧭 Visualization — publish to drivetrain's Field2d for AdvantageScope
        // drivetrain.getField().getObject("RobotPose").setPose(robotPose);
        // drivetrain.getField().getObject("NearestTag").setPose(tagPose2d);
        // drivetrain.getField().getObject("GoalPose").setPose(goalPose);

        return Optional.of(goalPose);
    }
}
