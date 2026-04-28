package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTracer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem.
 *
 * <p>The subsystem updates each camera once per loop, creates at most one accepted candidate per
 * camera, optionally fuses candidates from multiple cameras, and feeds the selected field estimate
 * into {@link RobotState}.
 */
public class Vision extends SubsystemBase {
  private final Camera[] cameras;

  /** Per-camera processing mode. */
  public enum VisionEstimationMode {
    /** Limelight botpose/Megatag1 style pose estimate. */
    MT1,

    /** Limelight Megatag2 pose estimate. Requires robot yaw to be sent before reading. */
    MT2,

    /** Single-tag fallback that uses vision XY and gyro/odometry yaw. */
    SINGLE_TAG_GYRO,

    /** Log-only tx/ty/ta mode. No estimator injection. */
    TX_TY_TA
  }

  /** Optional yaw provider for MT2 and single-tag fallback fusion. */
  private Supplier<Rotation2d> yawSupplier = null;

  /**
   * Creates the vision subsystem.
   *
   * @param cameras cameras owned by this subsystem
   */
  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  /**
   * Provides an external yaw supplier.
   *
   * <p>The yaw is sent to cameras before each camera update, which keeps MT2 input snapshots and
   * candidate reads aligned to the current robot orientation.
   *
   * @param yawSupplier supplier of current robot yaw in field coordinates
   */
  public void setYawSupplier(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  /**
   * Updates each camera once, builds candidates from cached camera data, and feeds the estimator.
   */
  @Override
  public void periodic() {
    LoggedTracer.record("VisionStart");

    Rotation2d yawNow = yawSupplier != null ? yawSupplier.get() : null;
    List<VisionCandidate> candidates = new ArrayList<>();

    for (Camera camera : cameras) {
      publishYawToCamera(camera, yawNow);
      camera.periodic();

      buildCandidateForMode(camera, yawNow).ifPresent(candidates::add);

      Logger.recordOutput(
          "Vision/Camera/" + camera.getName() + "/Mode", camera.getVisionMode().toString());
    }

    injectCandidates(candidates);

    Logger.recordOutput("Vision/CandidateCount", candidates.size());
    LoggedTracer.record("Vision");
  }

  /**
   * Sends yaw to the camera before camera inputs are read.
   *
   * @param camera camera to update
   * @param yawNow latest robot yaw, or null if unavailable
   */
  private void publishYawToCamera(Camera camera, Rotation2d yawNow) {
    if (yawNow == null || camera.getIo() == null) {
      return;
    }

    camera.getIo().setRobotYawDegrees(yawNow.getDegrees());
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/YawPublishedDeg", yawNow.getDegrees());
  }

  /**
   * Builds one vision candidate from the selected camera mode.
   *
   * @param camera camera to process
   * @param yawNow latest robot yaw, or null if unavailable
   * @return accepted candidate, if available
   */
  private Optional<VisionCandidate> buildCandidateForMode(Camera camera, Rotation2d yawNow) {
    return switch (camera.getVisionMode()) {
      case MT1 -> pickFromPoseEstimate(camera, camera.getIo().readMT1(), true, yawNow);
      case MT2 -> pickFromPoseEstimate(camera, camera.getIo().readMT2(), false, yawNow);
      case SINGLE_TAG_GYRO -> pickSingleTagFallback(camera, yawNow);
      case TX_TY_TA -> {
        logTxTyTa(camera);
        yield Optional.empty();
      }
    };
  }

  /**
   * Injects the best available vision candidate into the field estimator.
   *
   * <p>When two or more candidates are available, the first two are fused. This preserves the
   * current two-camera behavior while avoiding extra camera reads.
   *
   * @param candidates accepted vision candidates for this loop
   */
  private void injectCandidates(List<VisionCandidate> candidates) {
    if (candidates.isEmpty()) {
      return;
    }

    if (candidates.size() >= 2) {
      feedFieldEstimate(fuse(candidates.get(0), candidates.get(1)));
      return;
    }

    feedFieldEstimate(candidates.get(0));
  }

  /**
   * Adds a fused or single candidate into the field estimator and logs useful artifacts.
   *
   * @param candidate candidate to feed
   */
  private void feedFieldEstimate(VisionCandidate candidate) {
    RobotState.getInstance()
        .addFieldVisionMeasurement(
            candidate.pose(),
            candidate.timestampSec(),
            candidate.xyStdDev(),
            candidate.rotStdDev());

    Logger.recordOutput("Vision/FusedPose", candidate.pose());
    Logger.recordOutput("Vision/FusedTimestamp", candidate.timestampSec());
    Logger.recordOutput("Vision/FusedXYStd", candidate.xyStdDev());
    Logger.recordOutput("Vision/FusedRotStd", candidate.rotStdDev());
    Logger.recordOutput("Vision/FusedTrustYaw", candidate.trustYaw());
  }

  /**
   * Builds a candidate from a Limelight pose estimate.
   *
   * @param camera camera that produced the estimate
   * @param poseEstimate optional pose estimate
   * @param usePrimaryCoefficient true to use the camera primary XY coefficient
   * @param yawNow fused yaw to use for single-tag estimates, may be null
   * @return accepted candidate, if available
   */
  private Optional<VisionCandidate> pickFromPoseEstimate(
      Camera camera,
      Optional<LimelightHelpers.PoseEstimate> poseEstimate,
      boolean usePrimaryCoefficient,
      Rotation2d yawNow) {

    if (poseEstimate.isEmpty()) {
      Logger.recordOutput("Vision/Camera/" + camera.getName() + "/PoseEstimatePresent", false);
      return Optional.empty();
    }

    var estimate = poseEstimate.get();
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/PoseEstimatePresent", true);

    if (estimate.pose == null || estimate.tagCount <= 0) {
      Logger.recordOutput("Vision/Rejected/" + camera.getName() + "/MissingPoseOrTags", true);
      return Optional.empty();
    }

    Pose2d pose = estimate.pose;
    boolean poseInField = isPoseWithinField(pose);
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/PoseInField", poseInField);

    if (!poseInField) {
      Logger.recordOutput("Vision/Rejected/OutOfField/" + camera.getName(), pose);
      return Optional.empty();
    }

    if (!passesSingleTagQualityChecks(camera, estimate)) {
      return Optional.empty();
    }

    double xyStdDev = calculateXYStdDev(camera, estimate, usePrimaryCoefficient);
    boolean trustYaw = estimate.tagCount >= 2;
    Pose2d acceptedPose =
        !trustYaw && yawNow != null ? new Pose2d(pose.getTranslation(), yawNow) : pose;

    boolean allowYawUpdate = DriverStation.isDisabled();
    double rotStdDev = allowYawUpdate && trustYaw ? 1.0 : 9999.0;

    Logger.recordOutput("Vision/CandidatePose/" + camera.getName(), acceptedPose);
    Logger.recordOutput("Vision/CandidateXYStd/" + camera.getName(), xyStdDev);
    Logger.recordOutput("Vision/CandidateRotStd/" + camera.getName(), rotStdDev);
    Logger.recordOutput("Vision/CandidateTags/" + camera.getName(), estimate.tagCount);
    Logger.recordOutput("Vision/CandidateTrustYaw/" + camera.getName(), trustYaw);

    return Optional.of(
        new VisionCandidate(
            acceptedPose, estimate.timestampSeconds, xyStdDev, trustYaw, rotStdDev));
  }

  /**
   * Applies single-tag ambiguity and area gates.
   *
   * @param camera camera that produced the estimate
   * @param estimate pose estimate to validate
   * @return true when the estimate passes quality checks
   */
  private boolean passesSingleTagQualityChecks(
      Camera camera, LimelightHelpers.PoseEstimate estimate) {
    if (estimate.tagCount != 1 || estimate.rawFiducials == null || estimate.rawFiducials.length < 1) {
      return true;
    }

    double ambiguity = estimate.rawFiducials[0].ambiguity;
    boolean ambiguityOk = ambiguity <= 0.5;
    boolean areaOk = estimate.avgTagArea >= 0.25;

    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/Ambiguity", ambiguity);
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/AmbiguityOK", ambiguityOk);
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/AvgTagArea", estimate.avgTagArea);
    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/TagAreaOK", areaOk);

    if (!ambiguityOk) {
      Logger.recordOutput("Vision/Rejected/HighAmbiguity/" + camera.getName(), ambiguity);
      return false;
    }

    if (!areaOk) {
      Logger.recordOutput("Vision/Rejected/SmallTagArea/" + camera.getName(), estimate.avgTagArea);
      return false;
    }

    return true;
  }

  /**
   * Calculates the XY standard deviation for a pose estimate.
   *
   * @param camera camera that produced the estimate
   * @param estimate pose estimate to model
   * @param usePrimaryCoefficient true to use primary coefficient, false to use secondary
   * @return modeled XY standard deviation in meters
   */
  private double calculateXYStdDev(
      Camera camera, LimelightHelpers.PoseEstimate estimate, boolean usePrimaryCoefficient) {
    double distance = Math.max(0.01, estimate.avgTagDist);
    double modeled = Math.pow(Math.max(distance, 0.8), 3.5) / Math.max(1, estimate.tagCount);

    if (DriverStation.isAutonomous()) {
      modeled *= 2.0;
    }

    double coefficient =
        usePrimaryCoefficient
            ? camera.getPrimaryXYStandardDeviationCoefficient()
            : camera.getSecondaryXYStandardDeviationCoefficient();

    double xyStdDev = Math.max(coefficient * modeled, 0.04);

    Logger.recordOutput("Vision/Camera/" + camera.getName() + "/ModeledXYStd", xyStdDev);
    return xyStdDev;
  }

  /**
   * Single-tag fallback. Uses vision XY and replaces yaw with gyro/odometry yaw when available.
   *
   * @param camera camera to read
   * @param yawNow current fused yaw, may be null
   * @return accepted candidate, if available
   */
  private Optional<VisionCandidate> pickSingleTagFallback(Camera camera, Rotation2d yawNow) {
    Optional<LimelightHelpers.PoseEstimate> selected =
        camera
            .getIo()
            .readMT1()
            .filter(estimate -> estimate != null && estimate.tagCount == 1)
            .or(() ->
                camera
                    .getIo()
                    .readMT2()
                    .filter(estimate -> estimate != null && estimate.tagCount == 1));

    if (selected.isEmpty()) {
      return Optional.empty();
    }

    var estimate = selected.get();

    if (estimate.pose == null || !isPoseWithinField(estimate.pose)) {
      return Optional.empty();
    }

    if (!passesSingleTagQualityChecks(camera, estimate)) {
      return Optional.empty();
    }

    double distance = Math.max(0.01, estimate.avgTagDist);
    double modeled = Math.pow(Math.max(distance, 0.8), 3.5);

    if (DriverStation.isAutonomous()) {
      modeled *= 2.0;
    }

    double xyStdDev = Math.max(camera.getPrimaryXYStandardDeviationCoefficient() * modeled, 0.04);
    Pose2d acceptedPose =
        yawNow != null ? new Pose2d(estimate.pose.getTranslation(), yawNow) : estimate.pose;

    Logger.recordOutput("Vision/CandidatePose/" + camera.getName(), acceptedPose);
    Logger.recordOutput("Vision/CandidateXYStd/" + camera.getName(), xyStdDev);
    Logger.recordOutput("Vision/CandidateRotStd/" + camera.getName(), 9999.0);
    Logger.recordOutput("Vision/CandidateTags/" + camera.getName(), estimate.tagCount);
    Logger.recordOutput("Vision/CandidateTrustYaw/" + camera.getName(), false);

    return Optional.of(
        new VisionCandidate(
            acceptedPose, estimate.timestampSeconds, xyStdDev, false, 9999.0));
  }

  /**
   * Logs raw 2D alignment signals for operator/servo use. No estimator injection.
   *
   * @param camera camera to read from
   */
  private void logTxTyTa(Camera camera) {
    camera
        .getIo()
        .readTxTyTa()
        .ifPresent(
            target -> {
              Logger.recordOutput("Vision/Tx/" + camera.getName(), target.txDeg());
              Logger.recordOutput("Vision/Ty/" + camera.getName(), target.tyDeg());
              Logger.recordOutput("Vision/Ta/" + camera.getName(), target.taPct());
            });
  }

  /**
   * Simple field bounds gate using {@link FieldConstants}.
   *
   * @param pose field pose to test
   * @return true if pose is within the playable area with edge tolerance
   */
  private boolean isPoseWithinField(Pose2d pose) {
    final double edgeToleranceMeters = 0.10;
    double x = pose.getX();
    double y = pose.getY();

    return x >= edgeToleranceMeters
        && x <= FieldConstants.fieldLength - edgeToleranceMeters
        && y >= edgeToleranceMeters
        && y <= FieldConstants.fieldWidth - edgeToleranceMeters;
  }

  /**
   * Fuses two candidates using inverse-variance weighting in X/Y and heading blending when trusted.
   *
   * @param a first candidate
   * @param b second candidate
   * @return fused candidate
   */
  private VisionCandidate fuse(VisionCandidate a, VisionCandidate b) {
    if (b.timestampSec() < a.timestampSec()) {
      VisionCandidate temp = a;
      a = b;
      b = temp;
    }

    double aWeight = invVar(a.xyStdDev());
    double bWeight = invVar(b.xyStdDev());

    double x = (a.pose().getX() * aWeight + b.pose().getX() * bWeight) / (aWeight + bWeight);
    double y = (a.pose().getY() * aWeight + b.pose().getY() * bWeight) / (aWeight + bWeight);

    Rotation2d heading;
    if (a.trustYaw() && b.trustYaw()) {
      double cos =
          a.pose().getRotation().getCos() * aWeight
              + b.pose().getRotation().getCos() * bWeight;
      double sin =
          a.pose().getRotation().getSin() * aWeight
              + b.pose().getRotation().getSin() * bWeight;
      heading = new Rotation2d(cos, sin);
    } else if (a.trustYaw()) {
      heading = a.pose().getRotation();
    } else if (b.trustYaw()) {
      heading = b.pose().getRotation();
    } else {
      heading = yawSupplier != null ? yawSupplier.get() : b.pose().getRotation();
    }

    Pose2d fusedPose = new Pose2d(x, y, heading);
    double fusedStdDev = Math.sqrt(1.0 / (aWeight + bWeight));
    double rotStdDev = a.trustYaw() && b.trustYaw() ? fusedStdDev : 9999.0;

    Logger.recordOutput("Vision/Fuse/A_TrustYaw", a.trustYaw());
    Logger.recordOutput("Vision/Fuse/B_TrustYaw", b.trustYaw());
    Logger.recordOutput("Vision/Fuse/FusedXYStd", fusedStdDev);
    Logger.recordOutput("Vision/Fuse/FusedRotStd", rotStdDev);

    return new VisionCandidate(
        fusedPose, b.timestampSec(), fusedStdDev, a.trustYaw() && b.trustYaw(), rotStdDev);
  }

  /**
   * @param std standard deviation in meters
   * @return inverse variance
   */
  private double invVar(double std) {
    double clampedStd = Math.max(1e-6, std);
    return 1.0 / (clampedStd * clampedStd);
  }

  /** Immutable per-camera candidate accepted by vision gating. */
  private static record VisionCandidate(
      Pose2d pose,
      double timestampSec,
      double xyStdDev,
      boolean trustYaw,
      double rotStdDev) {}
}