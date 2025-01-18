package frc.robot.util;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.BuildConstants;

public class AprilTagMap {
  private static final AprilTagFieldLayout map;

  static {
    AprilTagFieldLayout mapLayout;

    // Load AprilTagLayout
    if (BuildConstants.USE_OFFICIAL_APRIL_TAG_LAYOUT) {
      // Use official map
      mapLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
      System.out.println("Loaded official 2025 AprilTag layout");
    } else {
      try {
        // Try to load fmap file
        mapLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("AprilTagMaps/ITCMap.fmap"));
        System.out.println("Loaded practice AprilTag layout");
      } catch (IOException e) {
        // Failed to load fmap file, use official one
        DriverStation.reportError(String.format("Failed to load fmap file: %s", e.getMessage()), false);
        mapLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        System.out.println("Loaded official 2025 AprilTag layout");
      }
    }

    map = mapLayout;
  }

  /**
   * @param tag Tag id
   * @return The 2d position of the tag on the field
   */
  public static Translation2d getTagPose(int tag) {
    Optional<Pose3d> pose = map.getTagPose(tag);

    if (pose.isEmpty()) {
      // Unknown tag
      DriverStation.reportWarning(String.format("Unknown tag request (id %d)", tag), false);
      return Translation2d.kZero;
    } else {
      return new Translation2d(pose.get().getX(), pose.get().getY());
    }
  }

  /** Utility class, do not allow instantiation */
  private AprilTagMap() {}
}
