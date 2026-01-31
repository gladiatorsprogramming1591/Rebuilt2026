package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTracer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that:
 *
 * <ul>
 *   <li>Builds at most one {@code VisionCandidate} per camera per loop (pre-fusing samples per
 *       camera to avoid jitter).
 *   <li>Supports modes: MT1, MT2, single-tag fallback (XY from vision + fused yaw), and tx/ty/ta
 *       logging-only.
 *   <li>Optionally provides yaw to Limelights for MT2.
 *   <li>Fuses cross-camera candidates using inverse-variance weighting (à la 254).
 * </ul>
 *
 * <h2>Source References (teams & files)</h2>
 *
 * <ul>
 *   <li>254 (inverse-variance fuse, MT1 preference, gating): {@code VisionSubsystem.java}, {@code
 *       VisionIOHardwareLimelight.java} <br>
 *       https://github.com/Team254/FRC-2025-Public
 *   <li>2910 (pinhole/trig distance + odometry+vision fusion pattern): {@code
 *       VisionIOLimelight.java} <br>
 *       https://github.com/FRCTeam2910/2025CompetitionRobot-Public
 *   <li>6328 (observation gating, distance^n / tagCount stddev modeling, Photon/Lime IO split):
 *       {@code Vision.java}, {@code VisionIOLimelight.java}, {@code VisionIOPhotonVision.java} <br>
 *       https://github.com/Mechanical-Advantage/RobotCode2025Public
 *   <li>1678 (subsystem shell, logging/timing patterns): {@code LimelightSubsystem}, {@code
 *       VisionIOLimelight} <br>
 *       https://github.com/frc1678/C2025-Public
 * </ul>
 */
public class Vision extends SubsystemBase {
  private final Camera[] cameras;

  /** Per-camera processing mode (matchable at runtime). */
  public enum VisionEstimationMode {
    /** Limelight "botpose" (Megatag1 style, multi-tag preferred). Source: 254 VisionSubsystem. */
    MT1,
    /** Limelight "orb" pose (Megatag2; requires publishing yaw). Source: 254 VisionIO HW. */
    MT2,
    /**
     * Single-tag fallback: use XY from vision but yaw from gyro/odometry. Source: 254
     * fuse-with-gyro path for single-tag.
     */
    SINGLE_TAG_GYRO,
    /**
     * Log-only: publish tx/ty/ta for alignment, no estimator injection. Source: 6328 exposes target
     * offsets while gating pose updates.
     */
    TX_TY_TA
  }

  /** Optional yaw provider for MT2 and single-tag fallback fusion (e.g., Pigeon2/odometry). */
  private Supplier<Rotation2d> yawSupplier = null;

  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  /**
   * Provide an external yaw supplier (gyro/odometry). This will:
   *
   * <ul>
   *   <li>Be forwarded to Limelight for MT2 via {@code robot_orientation_set} (254/6328 style).
   *   <li>Replace single-tag vision yaw when {@link VisionEstimationMode#SINGLE_TAG_GYRO} is used.
   * </ul>
   *
   * @param yawSupplier supplier of current robot yaw in field coordinates
   */
  public void setYawSupplier(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  /** Main loop: update cameras, build per-camera candidates, then cross-camera fuse & feed. */
  @Override
  public void periodic() {
    LoggedTracer.record("VisionStart");
    List<Rotation2d> mt1Yaws = new ArrayList<>();
    // If available, push yaw to MT2 cameras before we read (254/6328 pattern).
    Rotation2d yawNow = (yawSupplier != null) ? yawSupplier.get() : null;

    for (Camera cam : cameras) {
      cam.periodic();
      boolean okToSeedYaw =
          DriverStation.isDisabled(); // TODO: is this a good enough marker of when to seed yaw?

      if (yawNow != null && cam.getIo() != null) {
        // Limelight robot_orientation_set (required for MT2)
        // 254 VisionIOHardwareLimelight.setLLSettings()/orientation path; 6328 VisionIOLimelight
        cam.getIo().setRobotYawDegrees(yawNow.getDegrees());
      }

      Optional<LimelightHelpers.PoseEstimate> mt1Opt = cam.getIo().readMT1();
      if (mt1Opt.isPresent()) {
        var pe = mt1Opt.get();
        if (pe.pose != null && isPoseWithinField(pe.pose)) {
          if (pe.tagCount == 1
              && pe.rawFiducials != null
              && pe.rawFiducials.length >= 1
              && pe.rawFiducials[0].ambiguity < 0.2) {
            mt1Yaws.add(pe.pose.getRotation());
          }
        }
      }
    }

    // Build <=1 candidate per camera (pre-fuse per-camera samples; see 254 notes about jitter).
    List<VisionCandidate> candidates = new ArrayList<>();
    for (Camera cam : cameras) {
      // Logger.recordOutput(
      //     "Vision/Camera/" + cam.getName() + "/TrackingPose",
      //     new Pose3d(RobotState.getInstance().getRobotPoseOdometry())
      //         .transformBy(
      //             new Transform3d(
      //                 cam.getCameraOffset().getTranslation().minus(new Translation3d(0, 0, 0.5)),
      //                 cam.getCameraOffset().getRotation())));
      // Logger.recordOutput(
      //     "Vision/Camera/" + cam.getName() + "/Pose",
      //     new Pose3d(RobotState.getInstance().getRobotPoseOdometry())
      //         .transformBy(
      //             new Transform3d(
      //                 cam.getCameraOffset().getTranslation().minus(new Translation3d(0, 0, 0)),
      //                 cam.getCameraOffset().getRotation())));
      Optional<VisionCandidate> cand =
          switch (cam.getVisionMode()) {
            case MT1 -> pickFromPoseEstimate(cam, cam.getIo().readMT1(), true, yawNow);
            case MT2 -> pickFromPoseEstimate(cam, cam.getIo().readMT2(), false, yawNow);
            case SINGLE_TAG_GYRO -> pickSingleTagFallback(cam, yawNow);
            case TX_TY_TA -> {
              logTxTyTa(cam);
              yield Optional.empty();
            }
          };
      cand.ifPresent(candidates::add);
      boolean injectVision =
          Math.abs(RobotState.getInstance().getRobotPoseReef().getX())
              < Integer.MAX_VALUE; // flip this false -> true to test
      if (injectVision) {
        // Optional: send to reef-localizer, separate from full-field estimator (your
        // dual-estimators).
        if (cam.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)) {
          cand.filter(c -> FieldConstants.Reef.isReefOnly(c.tagIds()))
              .ifPresent(
                  c ->
                      RobotState.getInstance()
                          .addReefVisionMeasurement(c.pose(), c.timestampSec(), c.xyStdDev()));
        }
      }
    }
    RobotState.getInstance().seedYawFromVisionSamples(mt1Yaws, /* gain */ 0.4);

    // Cross-camera fuse (254-style inverse-variance weighting), then feed field estimator.
    boolean injectVision =
        Math.abs(RobotState.getInstance().getRobotPoseField().getX())
            < Integer.MAX_VALUE; // flip this false -> true to test
    if (injectVision) {
      if (candidates.size() >= 2) {
        VisionCandidate fused = fuse(candidates.get(0), candidates.get(1));
        feedFieldEstimate(fused);
      } else if (candidates.size() == 1) {
        feedFieldEstimate(candidates.get(0));
      }
    }
    LoggedTracer.record("Vision");
  }

  /**
   * Adds a fused (or single) candidate into the field estimator and logs useful artifacts.
   *
   * @param c candidate to feed
   */
  private void feedFieldEstimate(VisionCandidate c) {
    RobotState.getInstance().addFieldVisionMeasurement(c.pose(), c.timestampSec(), c.xyStdDev());
    Logger.recordOutput("Vision/FusedPose", c.pose());
    Logger.recordOutput("Vision/FusedTimestamp", c.timestampSec());
    Logger.recordOutput("Vision/FusedXYStd", c.xyStdDev());
  }

  /**
   * Build a candidate from a Limelight pose estimate (MT1 or MT2), applying basic gating and
   * std-dev modeling:
   *
   * <ul>
   *   <li>Field bounds check like 6328.
   *   <li>For single-tag, reject high ambiguity like 6328.
   *   <li>Std-dev scales as {@code distance^3.5 / tagCount} (6328 pattern) with per-camera coeff;
   *       looser in auto (6328).
   *   <li>Yaw trusted on multi-tag; for single-tag use fused yaw if supplied (254’s single-tag
   *       idea).
   * </ul>
   *
   * @param cam the camera
   * @param peOpt optional pose estimate
   * @param mt1Primary whether this is the MT1 (primary) channel for coeff selection
   * @param yawNow fused yaw to use for single-tag (may be null)
   * @return present if accepted
   */
  private Optional<VisionCandidate> pickFromPoseEstimate(
      Camera cam,
      Optional<LimelightHelpers.PoseEstimate> peOpt,
      boolean mt1Primary,
      Rotation2d yawNow) {

    if (peOpt.isEmpty()) return Optional.empty();
    var pe = peOpt.get();
    if (pe.pose == null) return Optional.empty();
    if (pe.tagCount <= 0) return Optional.empty();

    // 6328-style field bounds gating
    Pose2d pose = pe.pose;
    if (!isPoseWithinField(pose)) {
      Logger.recordOutput("Vision/Rejected/OutOfField", pose);
      return Optional.empty();
    }

    // 6328 single-tag ambiguity gating
    if (pe.tagCount == 1 && pe.rawFiducials != null && pe.rawFiducials.length >= 1) {
      double ambiguity = pe.rawFiducials[0].ambiguity;
      if (ambiguity > 0.2) { // conservative default; tune per camera
        Logger.recordOutput("Vision/Rejected/HighAmbiguity", ambiguity);
        return Optional.empty();
      }

      if (pe.avgTagArea < 1.0) {
        return Optional.empty();
      }

      var priorPose = RobotState.getInstance().getRobotPoseField();
      if (pe.avgTagArea < 2.0) {
        double yawDiff =
            Math.abs(
                MathUtil.angleModulus(
                    priorPose.getRotation().getRadians() - pose.getRotation().getRadians()));

        if (yawDiff > Units.degreesToRadians(5.0)) {
          return Optional.empty();
        }
      }
    }

    // 6328-style std-dev model: (dist^3.5 / tagCount) * coeff; looser in auto.
    double dist = Math.max(0.01, pe.avgTagDist);
    double modeled = Math.pow(Math.max(dist, 0.8), 3.5) / Math.max(1, pe.tagCount);
    if (DriverStation.isAutonomous()) modeled *= 2.0;
    double coeff =
        mt1Primary
            ? cam.getPrimaryXYStandardDeviationCoefficient()
            : cam.getSecondaryXYStandardDeviationCoefficient();
    double xyStd = coeff * modeled;
    xyStd = Math.max(xyStd, 0.08); // 3 cm; tune 0.03–0.07

    // Exclusive‑tag filtering USE FOR AUTO ALIGN
    // var exclusiveTag = state.getExclusiveTag();
    // boolean hasExclusiveId =
    //         exclusiveTag.isPresent()
    //                 && java.util.Arrays.stream(poseEstimate.fiducialIds())
    //                         .anyMatch(id -> id == exclusiveTag.get());

    // if (exclusiveTag.isPresent() && !hasExclusiveId) {
    //   return Optional.empty();
    // }
    // Trust yaw for multi-tag; else replace yaw with fused yaw (254 single-tag + gyro fusion idea).
    boolean trustYaw = pe.tagCount >= 2;
    Pose2d out = (!trustYaw && yawNow != null) ? new Pose2d(pose.getTranslation(), yawNow) : pose;

    // Logging (1678/254 style telemetry hygiene)
    Logger.recordOutput("Vision/CandidatePose/" + cam.getName(), out);
    Logger.recordOutput("Vision/CandidateXYStd/" + cam.getName(), xyStd);
    Logger.recordOutput("Vision/CandidateTags/" + cam.getName(), pe.tagCount);
    int[] ids =
        (pe.rawFiducials == null)
            ? new int[0]
            : Arrays.stream(pe.rawFiducials).filter(f -> f != null).mapToInt(f -> f.id).toArray();
    return Optional.of(new VisionCandidate(out, pe.timestampSeconds, xyStd, trustYaw, ids));
  }

  /**
   * Single-tag fallback: prefer MT1 if single tag, else MT2 if single tag. XY from vision, yaw from
   * gyro/odometry if available.
   *
   * <p>Pattern source: 254’s "fuse with gyro" path for single-tag when MT multi-tag quality is low.
   *
   * @param cam the camera
   * @param isBlue alliance color for LL helper
   * @param yawNow current fused yaw (may be null)
   * @return present if accepted
   */
  private Optional<VisionCandidate> pickSingleTagFallback(Camera cam, Rotation2d yawNow) {
    Optional<LimelightHelpers.PoseEstimate> mt1 = cam.getIo().readMT1();
    Optional<LimelightHelpers.PoseEstimate> mt2 = cam.getIo().readMT2();

    Optional<LimelightHelpers.PoseEstimate> selected =
        mt1.filter(pe -> pe != null && pe.tagCount == 1)
            .or(() -> mt2.filter(pe -> pe != null && pe.tagCount > 1));

    if (selected.isEmpty()) return Optional.empty();
    var pe = selected.get();
    if (pe.pose == null) return Optional.empty();
    if (!isPoseWithinField(pe.pose)) return Optional.empty();

    // 6328-style std-dev model for one tag
    double dist = Math.max(0.01, pe.avgTagDist);
    double modeled = Math.pow(dist, 3.5) / 1.0;
    if (DriverStation.isAutonomous()) modeled *= 2.0;
    double xyStd = cam.getPrimaryXYStandardDeviationCoefficient() * modeled;

    Pose2d out = (yawNow != null) ? new Pose2d(pe.pose.getTranslation(), yawNow) : pe.pose;
    int[] ids =
        (pe.rawFiducials == null)
            ? new int[0]
            : java.util.Arrays.stream(pe.rawFiducials)
                .filter(f -> f != null)
                .mapToInt(f -> f.id)
                .toArray();
    return Optional.of(new VisionCandidate(out, pe.timestampSeconds, xyStd, false, ids));
  }

  /**
   * Log raw 2D alignment signals (tx/ty/ta) for operator/servo use. No estimator injection.
   *
   * <p>Source pattern: 6328 keeps tx/ty while gating pose addVisionMeasurement.
   *
   * @param cam camera to read from
   */
  private void logTxTyTa(Camera cam) {
    cam.getIo()
        .readTxTyTa()
        .ifPresent(
            t -> {
              Logger.recordOutput("Vision/Tx/" + cam.getName(), t.txDeg());
              Logger.recordOutput("Vision/Ty/" + cam.getName(), t.tyDeg());
              Logger.recordOutput("Vision/Ta/" + cam.getName(), t.taPct());
            });
  }

  /**
   * Simple field bounds gate using {@link FieldConstants}.
   *
   * <p>Source concept: 6328 Vision.java rejects poses outside field extents.
   *
   * @param pose field pose to test
   * @return true if pose (x,y) is within the playable area (with a small edge tolerance)
   */
  private boolean isPoseWithinField(Pose2d pose) {
    final double edgeTolMeters = 0.10; // tighten/loosen if needed
    double x = pose.getX();
    double y = pose.getY();
    return x >= edgeTolMeters
        && x <= (FieldConstants.fieldLength - edgeTolMeters)
        && y >= edgeTolMeters
        && y <= (FieldConstants.fieldWidth - edgeTolMeters);
  }

  /**
   * Fuse two candidates via inverse-variance weighting in X/Y, and cosine-sine blending for yaw
   * when both yaws are trusted.
   *
   * <p>Source: 254 {@code VisionSubsystem.fuseEstimates()} (adapted to our scalar XY σ model).
   *
   * @param a candidate A
   * @param b candidate B
   * @return fused candidate (newer timestamp)
   */
  private VisionCandidate fuse(VisionCandidate a, VisionCandidate b) {
    // Make b the newer one
    if (b.timestampSec() < a.timestampSec()) {
      var tmp = a;
      a = b;
      b = tmp;
    }

    double wAx = invVar(a.xyStdDev());
    double wBx = invVar(b.xyStdDev());
    double wAy = wAx, wBy = wBx; // single scalar σ for xy in RobotState API

    double x = (a.pose().getX() * wAx + b.pose().getX() * wBx) / (wAx + wBx);
    double y = (a.pose().getY() * wAy + b.pose().getY() * wBy) / (wAy + wBy);

    Rotation2d heading;
    if (a.trustYaw() && b.trustYaw()) {
      // 254: inverse-variance heading blend
      double wA = wAx, wB = wBx;
      double cos = a.pose().getRotation().getCos() * wA + b.pose().getRotation().getCos() * wB;
      double sin = a.pose().getRotation().getSin() * wA + b.pose().getRotation().getSin() * wB;
      heading = new Rotation2d(cos, sin);
    } else if (a.trustYaw()) {
      heading = a.pose().getRotation();
    } else if (b.trustYaw()) {
      heading = b.pose().getRotation();
    } else {
      // Neither yaw trusted: prefer gyro if available, else newer
      heading = (yawSupplier != null) ? yawSupplier.get() : b.pose().getRotation();
    }

    Pose2d fusedPose = new Pose2d(x, y, heading);
    double fusedStd = Math.sqrt(1.0 / (wAx + wBx));
    double t = b.timestampSec();

    // Log a 3-vector for downstream debuggers (254 shows covariance-ish logging)
    Matrix<N3, N1> fusedStdN3 =
        VecBuilder.fill(fusedStd, fusedStd, a.trustYaw() && b.trustYaw() ? fusedStd : 9999.0);
    Logger.recordOutput("Vision/Fuse/stdN3", fusedStdN3);

    int[] fusedIds =
        IntStream.concat(Arrays.stream(a.tagIds()), Arrays.stream(b.tagIds()))
            .distinct() // keep if you want unique IDs
            .toArray();

    return new VisionCandidate(fusedPose, t, fusedStd, a.trustYaw() && b.trustYaw(), fusedIds);
  }

  /**
   * @param std standard deviation (meters) for XY
   * @return inverse variance (1 / σ²) with a small floor
   */
  private double invVar(double std) {
    double s = Math.max(1e-6, std);
    return 1.0 / (s * s);
  }

  /** Immutable per-camera candidate (already pre-fused for that camera). */
  private static record VisionCandidate(
      Pose2d pose, double timestampSec, double xyStdDev, boolean trustYaw, int[] tagIds) {}
}
