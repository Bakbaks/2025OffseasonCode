package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private static final Set<Integer> ALLOWED_TAG_IDS = Set.of(6,7,8,11,10,9,17,18,19,20,21,22);


    public VisionAim(AprilTagFieldLayout sharedLayout) {
        this.fieldLayout = sharedLayout;
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
     * Computes a target pose near the nearest tag by applying a fixed offset.
     *
     * @param robotPose current robot pose
     * @param robotToGoal desired offset from the tag (e.g., 0.5 m away)
     * @return field-space goal pose
     */
    public Optional<Pose2d> computeNearestGoalPose(Pose2d robotPose, Transform2d robotToGoal) {
        Optional<Pose3d> nearestTag = getNearestTagPose(robotPose);
        if (nearestTag.isEmpty()) return Optional.empty();

        Pose2d tagPose2d = nearestTag.get().toPose2d();
        Pose2d goalPose = tagPose2d.transformBy(robotToGoal);

        return Optional.of(goalPose);
    }
}
