package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final CameraType cameraType;
  private final double horizontalFOV;
  private final double verticalFOV;
  private final double primaryXYStandardDeviationCoefficient;
  private final double secondaryXYStandardDeviationCoefficient;

  /**
   * Constructs a Limelight-backed camera IO wrapper.
   *
   * @param name logical camera name (e.g., {@code "left"}); will be prefixed with {@code
   *     "limelight-"}
   * @param cameraType camera model/config for FOV and std-dev coefficients
   */
  public CameraIOLimelight(String name, CameraType cameraType) {
    this.name = "limelight-" + name;
    this.cameraType = cameraType;
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;
  }

  /**
   * Populates the {@link CameraIOInputs} snapshot from NetworkTables and LimelightHelpers.
   *
   * @param inputs mutable container to fill for logging/telemetry
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    double hb = NetworkTableInstance.getDefault().getTable(this.name).getEntry("hb").getDouble(-1);
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

    if (connected) {
      x = Rotation2d.fromDegrees(LimelightHelpers.getTX(name));
      y = Rotation2d.fromDegrees(LimelightHelpers.getTY(name));
      tv = LimelightHelpers.getTV(name);
      count = LimelightHelpers.getTargetCount(name);

      var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

      if (mt2 != null) {
        avgDist = mt2.avgTagDist;
        primary = mt2.pose; // prefer MT2 in primary slot
        ts = mt2.timestampSeconds;
      }
      if (mt1 != null) {
        secondary = mt1.pose;
        ts = (ts == 0.0) ? mt1.timestampSeconds : ts;
      }

      tagId = LimelightHelpers.getFiducialID(name);
    }

    inputs.data =
        new CameraIOData(hb, connected, x, y, tv, count, avgDist, ts, primary, secondary, tagId);
  }

  /**
   * Determines if the camera is alive based on the heartbeat.
   *
   * @param heartbeat value of {@code nt:/<ll>/hb}, or {@code -1} if absent
   * @return true if connected and producing heartbeats
   */
  private boolean getIsConnected(double heartbeat) {
    return heartbeat != -1;
  }

  /**
   * @return Limelight table name (e.g., {@code "limelight-left"})
   */
  @Override
  public String getName() {
    return name;
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
    var pe = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    return Optional.ofNullable(pe);
  }

  /**
   * Reads a Megatag2-style pose estimate ({@code botpose_orb_*}). Requires publishing robot yaw.
   *
   * @return optional Limelight pose estimate (MT2)
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT2() {
    var pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.ofNullable(pe);
  }

  /**
   * Reads raw 2D alignment signals.
   *
   * @return {@link Target2D} with {@code tx} (deg), {@code ty} (deg), {@code ta} (% of image), or
   *     empty if no valid target
   */
  @Override
  public Optional<Target2D> readTxTyTa() {
    if (LimelightHelpers.getTV(name)) {
      return Optional.of(
          new Target2D(
              LimelightHelpers.getTX(name),
              LimelightHelpers.getTY(name),
              LimelightHelpers.getTA(name)));
    }
    return Optional.empty();
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
