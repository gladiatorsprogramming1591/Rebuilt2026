package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;

/**
 * {@link CameraIO} implementation for a Limelight unit.
 *
 * <p>This class performs the physical NetworkTables/Limelight reads once in
 * {@link #updateInputs(CameraIOInputs)}. The mode-specific read helpers return cached values from
 * the latest update so {@link Vision} does not reread the same data multiple times in one loop.
 */
public class CameraIOLimelight implements CameraIO {
  private static final double HEARTBEAT_STALE_SECONDS = 0.5;

  private final String name;
  private final String tableKey;
  private final CameraType cameraType;
  private final double horizontalFOV;
  private final double verticalFOV;
  private final double primaryXYStandardDeviationCoefficient;
  private final double secondaryXYStandardDeviationCoefficient;

  private Optional<LimelightHelpers.PoseEstimate> latestMT1 = Optional.empty();
  private Optional<LimelightHelpers.PoseEstimate> latestMT2 = Optional.empty();
  private Optional<Target2D> latestTarget2D = Optional.empty();

  private double lastHeartbeat = -1.0;
  private double lastHeartbeatChangeTimestamp = 0.0;

  /**
   * Constructs a Limelight-backed camera IO wrapper.
   *
   * @param name logical camera name, without the {@code limelight-} prefix
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
  }

  /**
   * Populates the {@link CameraIOInputs} snapshot from NetworkTables and LimelightHelpers.
   *
   * <p>This is the only method that should perform physical Limelight reads during the robot loop.
   *
   * @param inputs mutable container to fill for logging/telemetry
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    double heartbeat =
        NetworkTableInstance.getDefault().getTable(name).getEntry("hb").getDouble(-1.0);
    boolean connected = isHeartbeatConnected(heartbeat);

    Rotation2d xOffset = new Rotation2d();
    Rotation2d yOffset = new Rotation2d();
    boolean targetAquired = false;
    int totalTargets = 0;
    double averageDistance = 0.0;
    double frameTimestamp = 0.0;
    Pose2d primaryPose = new Pose2d();
    Pose2d secondaryPose = new Pose2d();
    double tagId = -1.0;

    latestMT1 = Optional.empty();
    latestMT2 = Optional.empty();
    latestTarget2D = Optional.empty();

    if (connected) {
      double tx = LimelightHelpers.getTX(name);
      double ty = LimelightHelpers.getTY(name);
      double ta = LimelightHelpers.getTA(name);

      xOffset = Rotation2d.fromDegrees(tx);
      yOffset = Rotation2d.fromDegrees(ty);
      targetAquired = LimelightHelpers.getTV(name);
      totalTargets = LimelightHelpers.getTargetCount(name);
      tagId = LimelightHelpers.getFiducialID(name);

      latestMT2 = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
      latestMT1 = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));

      if (targetAquired) {
        latestTarget2D = Optional.of(new Target2D(tx, ty, ta));
      }

      if (latestMT2.isPresent()) {
        var mt2 = latestMT2.get();
        averageDistance = mt2.avgTagDist;
        primaryPose = mt2.pose;
        frameTimestamp = mt2.timestampSeconds;
      }

      if (latestMT1.isPresent()) {
        var mt1 = latestMT1.get();
        secondaryPose = mt1.pose;

        if (frameTimestamp == 0.0) {
          averageDistance = mt1.avgTagDist;
          frameTimestamp = mt1.timestampSeconds;
        }
      }
    }

    inputs.data =
        new CameraIOData(
            heartbeat,
            connected,
            xOffset,
            yOffset,
            targetAquired,
            totalTargets,
            averageDistance,
            frameTimestamp,
            primaryPose,
            secondaryPose,
            tagId);
  }

  /**
   * Determines if the camera heartbeat is present and still updating.
   *
   * @param heartbeat latest heartbeat value
   * @return true when the camera is connected and the heartbeat has updated recently
   */
  private boolean isHeartbeatConnected(double heartbeat) {
    double now = Timer.getFPGATimestamp();

    if (heartbeat == -1.0) {
      return false;
    }

    if (heartbeat != lastHeartbeat) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeTimestamp = now;
      return true;
    }

    return now - lastHeartbeatChangeTimestamp <= HEARTBEAT_STALE_SECONDS;
  }

  /**
   * @return Limelight table name of device only, for example {@code limelight-left}
   */
  @Override
  public String getName() {
    return name;
  }

  /**
   * @return Limelight table key, for example {@code Vision/Cameras/limelight-left/}
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
   * @return configured camera type
   */
  @Override
  public CameraType getCameraType() {
    return cameraType;
  }

  /**
   * @return horizontal field of view in radians
   */
  @Override
  public double getHorizontalFOV() {
    return horizontalFOV;
  }

  /**
   * @return vertical field of view in radians
   */
  @Override
  public double getVerticalFOV() {
    return verticalFOV;
  }

  /**
   * @return coefficient for primary pose XY std-dev modeling
   */
  @Override
  public double getPrimaryXYStandardDeviationCoefficient() {
    return primaryXYStandardDeviationCoefficient;
  }

  /**
   * @return coefficient for secondary pose XY std-dev modeling
   */
  @Override
  public double getSecondaryXYStandardDeviationCoefficient() {
    return secondaryXYStandardDeviationCoefficient;
  }

  /**
   * Sets the active Limelight pipeline.
   *
   * @param pipeline pipeline index
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
   * Sets the camera pose relative to robot frame for Limelight internal transforms.
   *
   * @param cameraOffset camera-to-robot transform
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

  /**
   * Returns the cached Megatag1-style pose estimate from the latest input update.
   *
   * @return optional cached Limelight pose estimate
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT1() {
    return latestMT1;
  }

  /**
   * Returns the cached Megatag2-style pose estimate from the latest input update.
   *
   * @return optional cached Limelight pose estimate
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT2() {
    return latestMT2;
  }

  /**
   * Returns cached raw 2D alignment signals from the latest input update.
   *
   * @return tx/ty/ta target data, or empty if no target was detected
   */
  @Override
  public Optional<Target2D> readTxTyTa() {
    return latestTarget2D;
  }

  /**
   * Publishes the robot yaw to the Limelight, required for MT2.
   *
   * @param yawDeg robot yaw in field coordinates, degrees
   */
  @Override
  public void setRobotYawDegrees(double yawDeg) {
    LimelightHelpers.SetRobotOrientation(name, yawDeg, 0, 0, 0, 0, 0);
  }
}