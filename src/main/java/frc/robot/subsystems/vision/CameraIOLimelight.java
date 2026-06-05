package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;

/**
 * {@link CameraIO} implementation for a Limelight unit.
 *
 * <p>Provides typed accessors for MT1/MT2 pose estimates and basic tx/ty/ta readings, plus
 * configuration helpers (pipeline, valid tag filters, camera pose).
 */
public class CameraIOLimelight implements CameraIO {
  private final String name;
  private final String tableKey;
  private final CameraType cameraType;
  private final double horizontalFOV;
  private final double verticalFOV;
  private final double primaryXYStandardDeviationCoefficient;
  private final double secondaryXYStandardDeviationCoefficient;
  private final NetworkTableEntry heartbeatEntry;
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatChangeTimestamp = 0.0;

  private LimelightHelpers.PoseEstimate latestMT1Estimate = null;
  private LimelightHelpers.PoseEstimate latestMT2Estimate = null;
  private Target2D latestTarget2D = null;
  private Vision.VisionEstimationMode activeVisionMode = Vision.VisionEstimationMode.MT1;

  /**
   * Constructs a Limelight-backed camera IO wrapper.
   *
   * @param name logical camera name (e.g., {@code "left"}); will be prefixed with {@code
   *     "limelight-"}
   * @param cameraType camera model/config for FOV and std-dev coefficients
   */
  public CameraIOLimelight(String name, CameraType cameraType) {
    this.name = "limelight-" + name;
    this.tableKey = "Vision/Cameras/" + this.name + "/";
    this.cameraType = cameraType;
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;
    this.heartbeatEntry = NetworkTableInstance.getDefault().getTable(this.name).getEntry("hb");
  }

  /**
   * Populates the {@link CameraIOInputs} snapshot from NetworkTables and LimelightHelpers.
   *
   * @param inputs mutable container to fill for logging/telemetry
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    double hb = heartbeatEntry.getDouble(-1);
    boolean connected = getIsConnected(hb);

    Rotation2d x = new Rotation2d();
    Rotation2d y = new Rotation2d();
    boolean tv = false;
    int count = 0;
    double avgDist = 0.0;
    double ts = 0.0;
    var primary = new edu.wpi.first.math.geometry.Pose2d();
    var secondary = new edu.wpi.first.math.geometry.Pose2d();
    double tagId = -1.0;

    latestMT1Estimate = null;
    latestMT2Estimate = null;
    latestTarget2D = null;

    if (connected) {
      double[] t2d = LimelightHelpers.getT2DArray(name);
      if (t2d.length >= 10) {
        tv = t2d[0] == 1.0;
        count = (int) t2d[1];
        x = Rotation2d.fromDegrees(t2d[4]);
        y = Rotation2d.fromDegrees(t2d[5]);
        tagId = t2d[9];

        if (tv) {
          latestTarget2D = new Target2D(x.getDegrees(), y.getDegrees(), t2d[8]);
        }
      }

      if (shouldReadMT2()) {
        latestMT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      }
      if (shouldReadMT1()) {
        latestMT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      }

      if (latestMT2Estimate != null) {
        avgDist = latestMT2Estimate.avgTagDist;
        primary = latestMT2Estimate.pose; // prefer MT2 in primary slot
        ts = latestMT2Estimate.timestampSeconds;
      }
      if (latestMT1Estimate != null) {
        secondary = latestMT1Estimate.pose;
        if (latestMT2Estimate == null) {
          avgDist = latestMT1Estimate.avgTagDist;
          primary = latestMT1Estimate.pose;
          ts = latestMT1Estimate.timestampSeconds;
        } else {
          ts = (ts == 0.0) ? latestMT1Estimate.timestampSeconds : ts;
        }
      }
    }

    inputs.data =
        new CameraIOData(hb, connected, x, y, tv, count, avgDist, ts, primary, secondary, tagId);
  }

  /** Updates which Limelight pose arrays are refreshed each loop. */
  @Override
  public void setVisionMode(Vision.VisionEstimationMode mode) {
    activeVisionMode = mode != null ? mode : Vision.VisionEstimationMode.MT1;
  }

  /** Returns true when the current mode needs the MT1 pose array. */
  private boolean shouldReadMT1() {
    return activeVisionMode == Vision.VisionEstimationMode.MT1
        || activeVisionMode == Vision.VisionEstimationMode.SINGLE_TAG_GYRO;
  }

  /** Returns true when the current mode needs the MT2 pose array. */
  private boolean shouldReadMT2() {
    return activeVisionMode == Vision.VisionEstimationMode.MT2
        || activeVisionMode == Vision.VisionEstimationMode.SINGLE_TAG_GYRO;
  }

  /**
   * Determines if the camera is alive based on the heartbeat.
   *
   * @param heartbeat value of {@code nt:/<ll>/hb}, or {@code -1} if absent
   * @return true if connected and producing heartbeats
   */
  private boolean getIsConnected(double heartbeat) {
    double now = Timer.getTimestamp();

    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeTimestamp = now;
    }

    return heartbeat != -1.0 && now - lastHeartbeatChangeTimestamp < 0.5;
  }

  /**
   * @return Limelight table name of device only (e.g., {@code "limelight-left"})
   */
  @Override
  public String getName() {
    return name;
  }

  /**
   * @return Limelight table key (e.g., {@code "Vision/Cameras/limelight-left"})
   * @see edu.wpi.first.wpilibj.smartdashboard.SmartDashboard SmartDashboard
   */
  @Override
  public String getTableKey() {
    return tableKey;
  }

  @Override
  public String toString() {
    return name;
  }

  /**
   * Sets the active Limelight pipeline.
   *
   * @param pipeline pipeline index (0–9)
   */
  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  /**
   * Enables or pauses Limelight Rewind recording.
   *
   * @param enabled true to record into the rewind buffer, false to pause it
   */
  @Override
  public void setRewindEnabled(boolean enabled) {
    LimelightHelpers.setRewindEnabled(name, enabled);
  }

  /**
   * Triggers Limelight Rewind to save a buffered clip.
   *
   * @param durationSeconds amount of recent buffered video to save
   */
  @Override
  public void triggerRewindCapture(double durationSeconds) {
    LimelightHelpers.triggerRewindCapture(name, durationSeconds);
  }

  /**
   * Overrides valid AprilTag IDs for localization on the current pipeline.
   *
   * @param validIds list of accepted tag IDs
   */
  @Override
  public void setValidTags(int... validIds) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, validIds);
  }

  /**
   * Sets the camera pose relative to robot frame (for Limelight internal transforms).
   *
   * @param cameraOffset camera-to-robot transform (meters/radians)
   */
  @Override
  public void setCameraOffset(Transform3d cameraOffset) {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        cameraOffset.getX(),
        cameraOffset.getY(),
        cameraOffset.getZ(),
        Units.radiansToDegrees(cameraOffset.getRotation().getX()),
        Units.radiansToDegrees(cameraOffset.getRotation().getY()),
        Units.radiansToDegrees(cameraOffset.getRotation().getZ()));
  }

  // MT1 / MT2 reads and tx-ty-ta

  /**
   * Reads a Megatag1-style pose estimate ({@code botpose_*}).
   *
   * @return optional Limelight pose estimate
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT1() {
    return Optional.ofNullable(latestMT1Estimate);
  }

  /**
   * Reads a Megatag2-style pose estimate ({@code botpose_orb_*}). Requires publishing robot yaw.
   *
   * @return optional Limelight pose estimate (MT2)
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT2() {
    return Optional.ofNullable(latestMT2Estimate);
  }

  /**
   * Reads raw 2D alignment signals.
   *
   * @return {@link Target2D} with {@code tx} (deg), {@code ty} (deg), {@code ta} (% of image), or
   *     empty if no valid target
   */
  @Override
  public Optional<Target2D> readTxTyTa() {
    return Optional.ofNullable(latestTarget2D);
  }

  /**
   * Publishes the robot yaw to the Limelight (required for MT2).
   *
   * @param yawDeg robot yaw in field coordinates, degrees
   */
  @Override
  public void setRobotYawDegrees(double yawDeg) {
    LimelightHelpers.SetRobotOrientation(name, yawDeg, 0, 0, 0, 0, 0);
  }
}
