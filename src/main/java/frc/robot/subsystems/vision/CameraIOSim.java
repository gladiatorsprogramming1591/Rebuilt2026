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
 * <p>This class mirrors the behavior and data contract of {@link CameraIOLimelight} so the rest of
 * the vision stack (modes MT1/MT2/tx-ty-ta and logging) can run unchanged in simulation.
 *
 * <p>Design notes:
 *
 * <ul>
 *   <li>Populates {@link CameraIOInputs#data} exactly like {@code CameraIOLimelight} does.
 *   <li>Provides {@link #readMT1(boolean)} / {@link #readMT2(boolean)} by fabricating {@link
 *       LimelightHelpers.PoseEstimate} objects from Photon results.
 *   <li>Provides {@link #readTxTyTa()} using the best target's yaw/pitch/area.
 *   <li>{@link #setRobotYawDegrees(double)} is a no-op in sim (Photon doesn't need it).
 * </ul>
 *
 * <p>Implementation reference: Team 254's public 2025 code sim pattern with PhotonVision.
 */
public class CameraIOSim implements CameraIO {
  /** Single shared Photon vision "world" for all simulated cameras. */
  private static VisionSystemSim sharedVisionSystem = null;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  private final String nameLl; // "limelight-<name>" to match logging & helpers
  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;

  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> fieldToRobotSupplier;

  // Cache latest batch so readMT1/readMT2/readTxTyTa can reuse without refetching
  private List<PhotonPipelineResult> latestResults = List.of();

  private final AprilTagFieldLayout tagLayout;

  /**
   * Computes the diagonal field of view from horizontal and vertical FOVs.
   *
   * <p>PhotonVision's {@link org.photonvision.simulation.SimCameraProperties#setCalibration(int,
   * int, edu.wpi.first.math.geometry.Rotation2d)} expects a <em>diagonal</em> FOV. Given horizontal
   * and vertical FOVs in radians, this method derives the diagonal using:
   *
   * <pre>
   * diag = 2 * atan( sqrt( tan(h/2)^2 + tan(v/2)^2 ) )
   * </pre>
   *
   * @param horizFovRad horizontal FOV in radians (from {@code CameraType.horizontalFOV})
   * @param vertFovRad vertical FOV in radians (from {@code CameraType.verticalFOV})
   * @return diagonal FOV in radians for PhotonVision calibration
   */
  private static double diagonalFovRad(double horizFovRad, double vertFovRad) {
    double th = Math.tan(horizFovRad * 0.5);
    double tv = Math.tan(vertFovRad * 0.5);
    return 2.0 * Math.atan(Math.sqrt(th * th + tv * tv));
  }

  /**
   * Simple container describing a reasonable simulation preset for a given camera model.
   *
   * <p>These values are not strict hardware specs—just practical defaults that make the
   * PhotonVision simulator behave similarly to the physical device (resolution, FPS, latency,
   * exposure, and a minimum target area threshold to reduce noise).
   */
  private static class SimPreset {
    /** Image width in pixels. */
    final int width;
    /** Image height in pixels. */
    final int height;
    /** Simulated frames per second. */
    final int fps;
    /** Mean pipeline latency in milliseconds. */
    final double avgLatencyMs;
    /** Latency standard deviation in milliseconds. */
    final double latencyStdMs;
    /** Exposure time in milliseconds. */
    final double exposureMs;
    /** Minimum target area in pixels before a detection is considered valid. */
    final double minAreaPx;

    /**
     * @param w image width (px)
     * @param h image height (px)
     * @param fps frames per second
     * @param avg mean latency (ms)
     * @param std latency standard deviation (ms)
     * @param exp exposure time (ms)
     * @param minAreaPx minimum pixel area for a valid target
     */
    SimPreset(int w, int h, int fps, double avg, double std, double exp, double minAreaPx) {
      this.width = w;
      this.height = h;
      this.fps = fps;
      this.avgLatencyMs = avg;
      this.latencyStdMs = std;
      this.exposureMs = exp;
      this.minAreaPx = minAreaPx;
    }
  }

  /**
   * Returns a reasonable PhotonVision simulation preset for a given {@link CameraType}.
   *
   * <p>Values are conservative and intended to be tweaked. They keep sim behavior aligned with the
   * physical device class (Limelight 2+, 3/3G, 4).
   *
   * @param type the physical camera model used on the robot
   * @return a {@link SimPreset} describing recommended sim calibration and timing
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
   * @param name logical camera name, e.g. {@code "left"} (full name becomes {@code
   *     "limelight-left"})
   * @param cameraType camera model (FOV + std-dev coefficients)
   * @param robotToCamera transform from robot frame to this camera
   * @param fieldLayout AprilTag field layout to add to the shared sim world (only added once)
   * @param fieldToRobotSupplier supplier of the current field→robot pose for updating the sim world
   */
  public CameraIOSim(
      String name,
      CameraType cameraType,
      Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout,
      Supplier<Pose2d> fieldToRobotSupplier) {

    this.nameLl = "limelight-" + name;
    this.cameraType = cameraType;
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;

    this.robotToCamera = robotToCamera;
    this.fieldToRobotSupplier = fieldToRobotSupplier;

    // Build / reuse world
    if (sharedVisionSystem == null) {
      sharedVisionSystem = new VisionSystemSim("vision-world");
      if (fieldLayout != null) {
        sharedVisionSystem.addAprilTags(fieldLayout);
      }
    }

    this.tagLayout = fieldLayout;

    // Create Photon camera + sim properties using your CameraType
    this.camera = new PhotonCamera(name);

    double diagFovRad = diagonalFovRad(this.horizontalFOV, this.verticalFOV);
    SimPreset preset = presetFor(this.cameraType);

    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(preset.width, preset.height, Rotation2d.fromRadians(diagFovRad));
    props.setCalibError(0.35, 0.5);
    props.setFPS(preset.fps);
    props.setAvgLatencyMs(preset.avgLatencyMs);
    props.setLatencyStdDevMs(preset.latencyStdMs);
    props.setExposureTimeMs(preset.exposureMs);

    this.cameraSim = new PhotonCameraSim(this.camera, props);
    this.cameraSim.setMinTargetAreaPixels((int) preset.minAreaPx);
    this.cameraSim.enableRawStream(false);
    this.cameraSim.enableProcessedStream(false);
    this.cameraSim.enableDrawWireframe(false);

    sharedVisionSystem.addCamera(this.cameraSim, robotToCamera);
  }

  /**
   * Refreshes the {@link CameraIOInputs} snapshot from PhotonVision simulation.
   *
   * <p>Behavior matches {@link CameraIOLimelight#updateInputs(CameraIOInputs)}:
   *
   * <ul>
   *   <li>Heartbeat is non -1 when connected (sim uses current FPGA time).
   *   <li>Primary pose prefers multi-tag (MT2-like). Secondary pose is single-tag (MT1-like).
   *   <li>tx/ty/ta come from the best target when available.
   *   <li>Timestamps approximate capture time using result latency.
   * </ul>
   *
   * @param inputs mutable container to fill for logging/telemetry
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    LoggedTracer.record("VisionInputs");
    // Update world from latest robot pose
    Pose2d fieldToRobot = fieldToRobotSupplier.get();
    if (fieldToRobot != null) {
      sharedVisionSystem.update(fieldToRobot);
      Logger.recordOutput("Vision/Sim/RobotPose", fieldToRobot);
    }

    // Grab unread results
    latestResults = camera.getAllUnreadResults();

    // Synthesize snapshot (matches CameraIOLimelight fields)
    double heartbeat = Timer.getFPGATimestamp(); // non -1 => connected
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

    PhotonPipelineResult latest =
        latestResults.isEmpty() ? null : latestResults.get(latestResults.size() - 1);

    if (latest != null && latest.hasTargets()) {
      targetAquired = true;
      totalTargets = latest.getTargets().size();

      PhotonTrackedTarget best = latest.getBestTarget();
      xOffset = Rotation2d.fromDegrees(best.getYaw());
      yOffset = Rotation2d.fromDegrees(best.getPitch());
      tagID = best.getFiducialId();

      averageDistance = best.getBestCameraToTarget().getTranslation().getNorm();

      // Primary (MT2-like, multi-tag preferred)
      primaryPose = estimateFieldPose(latest, /*preferMultiTag=*/ true).orElse(primaryPose);

      // Secondary (MT1-like, single-tag)
      secondaryPose = estimateFieldPose(latest, /*preferMultiTag=*/ false).orElse(secondaryPose);

      // Approx capture time
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
   * @return {@code "limelight-<name>"} to match hardware naming and logging.
   */
  @Override
  public String getName() {
    return nameLl;
  }

  // -----------------------------
  // Optional helpers for Vision modes
  // -----------------------------

  /**
   * Reads a Megatag1-style pose estimate generated from sim data.
   *
   * @return optional fabricated {@link LimelightHelpers.PoseEstimate}
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT1() {
    PhotonPipelineResult latest =
        latestResults.isEmpty() ? null : latestResults.get(latestResults.size() - 1);
    if (latest == null || !latest.hasTargets()) return Optional.empty();
    return Optional.ofNullable(buildPoseEstimate(latest, /*treatAsMT2=*/ false));
  }

  /**
   * Reads a Megatag2-style pose estimate generated from sim data.
   *
   * @return optional fabricated {@link LimelightHelpers.PoseEstimate}
   */
  @Override
  public Optional<LimelightHelpers.PoseEstimate> readMT2() {
    PhotonPipelineResult latest =
        latestResults.isEmpty() ? null : latestResults.get(latestResults.size() - 1);
    if (latest == null || !latest.hasTargets()) return Optional.empty();
    return Optional.ofNullable(buildPoseEstimate(latest, /*treatAsMT2=*/ true));
  }

  /**
   * Reads raw 2D alignment info (tx/ty/ta) from the best target, if present.
   *
   * @return {@link Target2D} or {@link Optional#empty()}
   */
  @Override
  public Optional<Target2D> readTxTyTa() {
    PhotonPipelineResult latest =
        latestResults.isEmpty() ? null : latestResults.get(latestResults.size() - 1);
    if (latest == null || !latest.hasTargets()) return Optional.empty();

    PhotonTrackedTarget best = latest.getBestTarget();
    return Optional.of(new Target2D(best.getYaw(), best.getPitch(), best.getArea()));
  }

  /**
   * No-op in simulation (PhotonVision doesn't need robot yaw injection).
   *
   * @param yawDeg robot yaw in degrees
   */
  @Override
  public void setRobotYawDegrees(double yawDeg) {
    // Intentionally empty in sim
  }

  // -----------------------------
  // Configuration helpers (no-ops in sim, kept for parity with hardware class)
  // -----------------------------

  /** No-op in sim. */
  @Override
  public void setPipeline(int pipeline) {}

  /** No-op in sim. */
  @Override
  public void setValidTags(int... validIds) {}

  /**
   * In sim, the camera transform is set at construction time when registering with the world. This
   * method is a no-op to keep parity with {@link CameraIOLimelight}.
   */
  @Override
  public void setCameraOffset(Transform3d cameraOffset) {}

  // -----------------------------
  // Internal helpers
  // -----------------------------

  /**
   * Builds a Limelight-like PoseEstimate from a Photon result.
   *
   * @param result the latest Photon pipeline result
   * @param treatAsMT2 if true, marks the estimate as "MT2" and prefers multi-tag data
   * @return fabricated {@link LimelightHelpers.PoseEstimate}, or {@code null} if unavailable
   */
  private LimelightHelpers.PoseEstimate buildPoseEstimate(
      PhotonPipelineResult result, boolean treatAsMT2) {

    Optional<Pose2d> pose2d = estimateFieldPose(result, /*preferMultiTag=*/ treatAsMT2);
    if (pose2d.isEmpty()) return null;

    // Tag stats + raw fiducials
    int tagCount = result.getTargets().size();
    double avgDist = 0.0;
    double avgArea = 0.0;
    List<LimelightHelpers.RawFiducial> raws = new ArrayList<>();
    for (PhotonTrackedTarget t : result.getTargets()) {
      double distCamToTag = t.getBestCameraToTarget().getTranslation().getNorm();
      avgDist += distCamToTag;
      avgArea += t.getArea();
      Logger.recordOutput(
          "Vision/Camera/" + getName() + "/TagFound/" + t.fiducialId,
          tagLayout.getTagPose(t.fiducialId).get().toPose2d());
      raws.add(
          new LimelightHelpers.RawFiducial(
              t.getFiducialId(),
              t.getYaw(), // txnc (deg)
              t.getPitch(), // tync (deg)
              t.getArea(), // ta (% image)
              distCamToTag, // distToCamera (m)
              Math.max(
                  0.0,
                  distCamToTag - robotToCamera.getTranslation().getNorm()), // approx distToRobot
              t.getPoseAmbiguity()));
    }
    if (tagCount > 0) {
      avgDist /= tagCount;
      avgArea /= tagCount;
    }

    double tsCapture = result.getTimestampSeconds();

    return new LimelightHelpers.PoseEstimate(
        pose2d.get(),
        tsCapture,
        0,
        tagCount,
        /*tagSpan=*/ 0.0,
        avgDist,
        avgArea,
        raws.toArray(new LimelightHelpers.RawFiducial[0]),
        /*isMegaTag2=*/ treatAsMT2);
  }

  /**
   * Estimates field→robot pose from a Photon result.
   *
   * <p>If a multi-tag solution exists (and preferred), uses the multi-tag field→camera pose and
   * composes it with {@code camera→robot}. Otherwise, falls back to single-tag back-projection
   * using the best target and its fiducial pose.
   *
   * @param result photon pipeline result
   * @param preferMultiTag whether to use multi-tag when available
   * @return field→robot {@link Pose2d}, if resolvable
   */
  private Optional<Pose2d> estimateFieldPose(PhotonPipelineResult result, boolean preferMultiTag) {
    Pose3d fieldToRobot = null;
    preferMultiTag = true;

    if (preferMultiTag && result.getMultiTagResult().isPresent()) {
      // field->camera from multi-tag
      Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
      fieldToRobot =
          new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
              .transformBy(robotToCamera.inverse());
    } else if (result.hasTargets()) {
      PhotonTrackedTarget best = result.getBestTarget();

      Optional<Pose3d> fieldToTag =
          (tagLayout != null) ? tagLayout.getTagPose(best.getFiducialId()) : Optional.empty();
      if (fieldToTag.isPresent()) {
        Transform3d cameraToTarget = best.getBestCameraToTarget();
        Pose3d fieldToCamera = fieldToTag.get().transformBy(cameraToTarget.inverse());
        fieldToRobot = fieldToCamera.transformBy(robotToCamera.inverse());
      }
    }

    return (fieldToRobot == null) ? Optional.empty() : Optional.of(fieldToRobot.toPose2d());
  }
}
