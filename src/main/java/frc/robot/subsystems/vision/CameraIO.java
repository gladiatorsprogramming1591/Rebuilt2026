package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utilities.LimelightHelpers;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for a vision camera. Implementations (e.g., Limelight) should fill {@link
 * CameraIOInputs} and provide optional helpers for MT1/MT2 and tx/ty/ta.
 */
public interface CameraIO {
  /**
   * Logged snapshot of camera inputs for a single loop.
   *
   * <p>Use {@link #updateInputs(CameraIOInputs)} to populate the structure once per cycle.
   */
  @AutoLog
  public static class CameraIOInputs {
    /** Latest sensor/measurement packet for logging & accessors. */
    public CameraIOData data =
        new CameraIOData(
            -1.0, // currentHeartbeat
            false, // isConnected
            new Rotation2d(), // xOffset
            new Rotation2d(), // yOffset
            false, // targetAquired
            0, // totalTargets
            0.0, // averageDistance
            0.0, // frameTimestamp
            new Pose2d(), // primaryPose
            new Pose2d(), // secondaryPose
            -1.0 // tagIDOfInterest
            );
  }

  /**
   * Immutable data container for per-cycle camera state.
   *
   * @param currentHeartbeat camera heartbeat value (-1 if missing)
   * @param isConnected true if camera is producing valid data
   * @param xOffset tx in degrees wrapped as {@link Rotation2d}
   * @param yOffset ty in degrees wrapped as {@link Rotation2d}
   * @param targetAquired whether a target is currently detected
   * @param totalTargets number of detected targets
   * @param averageDistance average detected tag distance (meters)
   * @param frameTimestamp capture timestamp (seconds, robot epoch)
   * @param primaryPose primary pose estimate (e.g., MT2 preferred)
   * @param secondaryPose secondary pose estimate (e.g., MT1)
   * @param tagIDOfInterest last seen fiducial id (or -1)
   */
  public static record CameraIOData(
      double currentHeartbeat,
      boolean isConnected,
      Rotation2d xOffset,
      Rotation2d yOffset,
      boolean targetAquired,
      int totalTargets,
      double averageDistance,
      double frameTimestamp,
      Pose2d primaryPose,
      Pose2d secondaryPose,
      double tagIDOfInterest) {}

  /**
   * Minimal 2D target signals (for alignment/driver aids).
   *
   * @param txDeg horizontal offset in degrees
   * @param tyDeg vertical offset in degrees
   * @param taPct target area as % of image
   */
  public static record Target2D(double txDeg, double tyDeg, double taPct) {}

  /**
   * Refreshes the {@link CameraIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(CameraIOInputs inputs) {}

  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(CameraIOInputs inputs) {
    return false;
  }

  /**
   * @return latest tx as a {@link Rotation2d} (deg)
   */
  public default Rotation2d getXOffset() {
    return new Rotation2d();
  }

  /**
   * @return latest ty as a {@link Rotation2d} (deg)
   */
  public default Rotation2d getYOffset() {
    return new Rotation2d();
  }

  /**
   * @return whether a target is currently detected
   */
  public default boolean getTargetAquired() {
    return false;
  }

  /**
   * @return number of detected targets
   */
  public default int getTotalTargets() {
    return 0;
  }

  /**
   * @return average detected distance (meters)
   */
  public default double getAverageDistance() {
    return 0.0;
  }

  /**
   * @return capture timestamp in seconds
   */
  public default double getFrameTimestamp() {
    return 0.0;
  }

  /**
   * @return primary pose estimate (may be empty pose if unavailable)
   */
  public default Pose2d getPrimaryPose() {
    return new Pose2d();
  }

  /**
   * @return secondary pose estimate (may be empty pose if unavailable)
   */
  public default Pose2d getSecondaryPose() {
    return new Pose2d();
  }

  /**
   * @return a camera-relative pose of interest (implementation-specific), if any
   */
  public default Pose3d getPoseOfInterest() {
    return new Pose3d();
  }

  /**
   * @return last seen fiducial ID, or -1 if unknown
   */
  public default double getTagIDOfInterest() {
    return -1;
  }

  /**
   * @return current pipeline index
   */
  public default long getPipeline() {
    return 0;
  }

  /**
   * @return human-readable device name (e.g., {@code "limelight-left"})
   */
  public default String getName() {
    return "";
  }

  /**
   * @return configured camera type for this IO
   */
  public default CameraType getCameraType() {
    return CameraType.DEFAULT;
  }

  /**
   * @return horizontal field of view in radians
   */
  public default double getHorizontalFOV() {
    return 0.0;
  }

  /**
   * @return vertical field of view in radians
   */
  public default double getVerticalFOV() {
    return 0.0;
  }

  /**
   * @return coefficient for primary pose XY std-dev modeling
   */
  public default double getPrimaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  /**
   * @return coefficient for secondary pose XY std-dev modeling
   */
  public default double getSecondaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  /**
   * Sets the active detection pipeline on the device.
   *
   * @param pipeline pipeline index
   */
  public default void setPipeline(int pipeline) {}

  /**
   * Restricts valid AprilTag IDs used for localization.
   *
   * @param validIds list of allowed tag IDs
   */
  public default void setValidTags(int... validIds) {}

  /**
   * Configures the camera's pose relative to robot frame.
   *
   * @param cameraOffset camera-to-robot transform
   */
  public default void setCameraOffset(Transform3d cameraOffset) {}

  // NEW: specialized reads to support MT1 / MT2 / tx-ty-ta modes

  /**
   * Reads an MT1 (botpose) estimate in the requested alliance frame.
   *
   * @return optional pose estimate
   */
  public default Optional<LimelightHelpers.PoseEstimate> readMT1() {
    return Optional.empty();
  }

  /**
   * Reads an MT2 (orb) estimate in the requested alliance frame.
   *
   * @return optional pose estimate
   */
  public default Optional<LimelightHelpers.PoseEstimate> readMT2() {
    return Optional.empty();
  }

  /**
   * Reads raw 2D target info for alignment (no pose).
   *
   * @return {@link Target2D} with tx/ty/ta, or empty if no target
   */
  public default Optional<Target2D> readTxTyTa() {
    return Optional.empty();
  }

  // NEW: for MT2 – allow Vision to publish robot yaw to the camera

  /**
   * Publishes robot yaw (degrees) so the camera can incorporate it (e.g., MT2).
   *
   * @param yawDeg robot yaw in field coordinates, degrees
   */
  public default void setRobotYawDegrees(double yawDeg) {}
}
