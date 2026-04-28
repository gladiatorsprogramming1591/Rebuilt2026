package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTracer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * {@link CameraIO} simulation using PhotonVision's {@link VisionSystemSim}.
 *
 * <p>This mirrors the real Limelight IO contract closely enough that the rest of the vision stack can
 * run unchanged in simulation.
 */
public class CameraIOSim implements CameraIO {
  /** Single shared Photon vision world for all simulated cameras. */
  private static VisionSystemSim sharedVisionSystem = null;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  private final String nameLl;

  @Getter private final String tableKey;
  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;

  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> fieldToRobotSupplier;
  private final AprilTagFieldLayout tagLayout;

  private List<PhotonPipelineResult> latestResults = List.of();

  /**
   * Computes the diagonal field of view from horizontal and vertical FOVs.
   *
   * @param horizFovRad horizontal FOV in radians
   * @param vertFovRad vertical FOV in radians
   * @return diagonal FOV in radians for PhotonVision calibration
   */
  private static double diagonalFovRad(double horizFovRad, double vertFovRad) {
    double th = Math.tan(horizFovRad * 0.5);
    double tv = Math.tan(vertFovRad * 0.5);
    return 2.0 * Math.atan(Math.sqrt(th * th + tv * tv));
  }

  /** Simulation preset for a camera model. */
  private static class SimPreset {
    final int width;
    final int height;
    final int fps;
    final double avgLatencyMs;
    final double latencyStdMs;
    final double exposureMs;
    final double minAreaPx;

    SimPreset(
        int width,
        int height,
        int fps,
        double avgLatencyMs,
        double latencyStdMs,
        double exposureMs,
        double minAreaPx) {
      this.width = width;
      this.height = height;
      this.fps = fps;
      this.avgLatencyMs = avgLatencyMs;
      this.latencyStdMs = latencyStdMs;
      this.exposureMs = exposureMs;
      this.minAreaPx = minAreaPx;
    }
  }

  /**
   * Returns a reasonable PhotonVision simulation preset for a given {@link CameraType}.
   *
   * @param type camera type
   * @return sim preset
   */
  private static SimPreset presetFor(CameraType type) {
    return switch (type) {
      case LIMELIGHT_2_PLUS -> new SimPreset(960, 720, 45, 24, 5, 1.0, 1000);
      case LIMELIGHT_3, LIMELIGHT_3G -> new SimPreset(1280, 800, 45, 20, 5, 0.65, 1000);
      case LIMELIGHT_4 -> new SimPreset(1280, 800, 45, 20, 5, 0.55, 1000);
      default -> new SimPreset(1280, 800, 45, 20, 5, 0.65, 1000);
    };
  }

  /**
   * Constructs a simulated camera.
   *
   * @param name logical camera name
   * @param cameraType camera model
   * @param robotToCamera transform from robot frame to this camera
   * @param fieldLayout AprilTag field layout
   * @param fieldToRobotSupplier supplier for current robot pose
   */
  public CameraIOSim(
      String name,
      CameraType cameraType,
      Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout,
      Supplier<Pose2d> fieldToRobotSupplier) {

    this.nameLl = "limelight-" + name;
    this.tableKey = "Vision/Cameras/" + this.nameLl + "/";
    this.cameraType = cameraType;
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;
    this.robotToCamera = robotToCamera;
    this.fieldToRobotSupplier = fieldToRobotSupplier;
    this.tagLayout = fieldLayout;

    if (sharedVisionSystem == null) {
      sharedVisionSystem = new VisionSystemSim("vision-world");

      if (fieldLayout != null) {
        sharedVisionSystem.addAprilTags(fieldLayout);
      }
    }

    camera = new PhotonCamera(name);

    SimPreset preset = presetFor(cameraType);
    double diagFovRad = diagonalFovRad(horizontalFOV, verticalFOV);

    SimCameraProperties properties = new SimCameraProperties();
    properties.setCalibration(preset.width, preset.height, Rotation2d.fromRadians(diagFovRad));
    properties.setCalibError(0.35, 0.5);
    properties.setFPS(preset.fps);
    properties.setAvgLatencyMs(preset.avgLatencyMs);
    properties.setLatencyStdDevMs(preset.latencyStdMs);
    properties.setExposureTimeMs(preset.exposureMs);

    cameraSim = new PhotonCameraSim(camera, properties);
    cameraSim.setMinTargetAreaPixels((int) preset.minAreaPx);
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableDrawWireframe(false);

    sharedVisionSystem.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Refreshes the {@link CameraIOInputs} snapshot from PhotonVision simulation.
   *
   * @param inputs mutable container to fill for logging/telemetry
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    LoggedTracer.record("VisionInputs");

    Pose2d fieldToRobot = fieldToRobotSupplier.get();
    if (fieldToRobot != null) {
      sharedVisionSystem.update(fieldToRobot);
      Logger.recordOutput("Vision/Sim/RobotPose", fieldToRobot);
    }

    latestResults = camera.getAllUnreadResults();

    double heartbeat = Timer.getFPGATimestamp();
    boolean isConnected = true;

    Rotation2d xOffset = new Rotation2d();
    Rotation2d yOffset = new Rotation2d();
    boolean targetAquired = false;
    int totalTargets = 0;
    double averageDistance = 0.0;
    double frameTimestamp = 0.0;
    Pose2d primaryPose = new Pose2d();
    Pose2d secondaryPose = new Pose2d();
    double tagID = -1.0;

    PhotonPipelineResult latest = getLatestResult().orElse(null);

    if (latest != null && latest.hasTargets()) {
      targetAquired = true;
      totalTargets = latest.getTargets().size();

      PhotonTrackedTarget best = latest.getBestTarget();
      xOffset = Rotation2d.fromDegrees(best.getYaw());
      yOffset = Rotation2d.fromDegrees(best.getPitch());
      tagID = best.getFiducialId();
      averageDistance = best.getBestCameraToTarget().getTranslation().getNorm();

      primaryPose = estimateFieldPose(latest, true).orElse(primaryPose);
      secondaryPose = estimateFieldPose(latest, false).orElse(secondaryPose);

      frameTimestamp = latest.getTimestampSeconds();
    }

    inputs.data =
        new CameraIOData(
            heartbeat,
            isConnected,
            xOffset,
            yOffset,
            targetAquired,
            totalTargets,
            averageDistance,
            frameTimestamp,
            primaryPose,
            secondaryPose,
            tagID);
  }

  /**
   * @return {@code limelight-<name>} to match hardware naming and logging
   */
  @Override
  public String getName() {
    return nameLl;
  }

  /**
   * Reads a Megatag1-style pose estimate generated from sim data.
   *
   * @return optional fabricated pose estimate
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT1() {
    return getLatestResult()
        .filter(PhotonPipelineResult::hasTargets)
        .map(result -> buildPoseEstimate(result, false));
  }

  /**
   * Reads a Megatag2-style pose estimate generated from sim data.
   *
   * @return optional fabricated pose estimate
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT2() {
    return getLatestResult()
        .filter(PhotonPipelineResult::hasTargets)
        .map(result -> buildPoseEstimate(result, true));
  }

  /**
   * Reads raw 2D alignment info from the best target.
   *
   * @return tx/ty/ta data, or empty if no target is available
   */
  @Override
  public Optional<Target2D> readTxTyTa() {
    return getLatestResult()
        .filter(PhotonPipelineResult::hasTargets)
        .map(
            result -> {
              PhotonTrackedTarget best = result.getBestTarget();
              return new Target2D(best.getYaw(), best.getPitch(), best.getArea());
            });
  }

  /**
   * No-op in simulation.
   *
   * @param yawDeg robot yaw in degrees
   */
  @Override
  public void setRobotYawDegrees(double yawDeg) {
    // PhotonVision sim does not need Limelight robot orientation injection.
  }

  /** No-op in sim. */
  @Override
  public void setPipeline(int pipeline) {}

  /** No-op in sim. */
  @Override
  public void setValidTags(int... validIds) {}

  /** No-op in sim. The transform is provided at construction time. */
  @Override
  public void setCameraOffset(Transform3d cameraOffset) {}

  /**
   * Returns the most recent unread result from the latest update.
   *
   * @return latest result, if available
   */
  private Optional<PhotonPipelineResult> getLatestResult() {
    if (latestResults.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(latestResults.get(latestResults.size() - 1));
  }

  /**
   * Builds a Limelight-like pose estimate from a Photon result.
   *
   * @param result latest Photon result
   * @param treatAsMT2 true when this should mimic MT2
   * @return fabricated pose estimate
   */
  private LimelightHelpers.PoseEstimate buildPoseEstimate(
      PhotonPipelineResult result, boolean treatAsMT2) {

    Optional<Pose2d> pose2d = estimateFieldPose(result, treatAsMT2);
    if (pose2d.isEmpty()) {
      return null;
    }

    int tagCount = result.getTargets().size();
    double avgDist = 0.0;
    double avgArea = 0.0;
    List<LimelightHelpers.RawFiducial> rawFiducials = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      double distCamToTag = target.getBestCameraToTarget().getTranslation().getNorm();
      avgDist += distCamToTag;
      avgArea += target.getArea();

      logTagPoseIfAvailable(target);

      rawFiducials.add(
          new LimelightHelpers.RawFiducial(
              target.getFiducialId(),
              target.getYaw(),
              target.getPitch(),
              target.getArea(),
              distCamToTag,
              Math.max(0.0, distCamToTag - robotToCamera.getTranslation().getNorm()),
              target.getPoseAmbiguity()));
    }

    if (tagCount > 0) {
      avgDist /= tagCount;
      avgArea /= tagCount;
    }

    return new LimelightHelpers.PoseEstimate(
        pose2d.get(),
        result.getTimestampSeconds(),
        0.0,
        tagCount,
        0.0,
        avgDist,
        avgArea,
        rawFiducials.toArray(new LimelightHelpers.RawFiducial[0]),
        treatAsMT2);
  }

  /**
   * Logs a tag pose only when the layout contains that tag ID.
   *
   * @param target tracked target
   */
  private void logTagPoseIfAvailable(PhotonTrackedTarget target) {
    if (tagLayout == null) {
      return;
    }

    tagLayout
        .getTagPose(target.getFiducialId())
        .ifPresent(
            tagPose ->
                Logger.recordOutput(
                    "Vision/Camera/" + getName() + "/TagFound/" + target.getFiducialId(),
                    tagPose.toPose2d()));
  }

  /**
   * Estimates field-to-robot pose from a Photon result.
   *
   * @param result photon pipeline result
   * @param preferMultiTag whether to use multi-tag when available
   * @return field-to-robot pose, if resolvable
   */
  private Optional<Pose2d> estimateFieldPose(PhotonPipelineResult result, boolean preferMultiTag) {
    Pose3d fieldToRobot = null;

    if (preferMultiTag && result.getMultiTagResult().isPresent()) {
      Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
      fieldToRobot =
          new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
              .transformBy(robotToCamera.inverse());
    } else if (result.hasTargets()) {
      PhotonTrackedTarget best = result.getBestTarget();
      Optional<Pose3d> fieldToTag =
          tagLayout != null ? tagLayout.getTagPose(best.getFiducialId()) : Optional.empty();

      if (fieldToTag.isPresent()) {
        Transform3d cameraToTarget = best.getBestCameraToTarget();
        Pose3d fieldToCamera = fieldToTag.get().transformBy(cameraToTarget.inverse());
        fieldToRobot = fieldToCamera.transformBy(robotToCamera.inverse());
      }
    }

    return fieldToRobot == null ? Optional.empty() : Optional.of(fieldToRobot.toPose2d());
  }
}