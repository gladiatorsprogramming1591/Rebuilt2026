package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import lombok.Getter;
import lombok.Setter;

/**
 * RobotState: localization + motion state only. - Field & Reef Pose Estimators (odometry + vision)
 * - Odometry-only pose for debugging - Timestamped fused-pose buffer for latency
 * compensation/prediction - Measured & desired chassis speeds (robot & field frames) - Pose reset
 * with gyro offset anchoring
 */
public final class RobotState {

  // ===== Singleton =====
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // ===== Kinematics / Estimators / Odometry =====
  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  /** Fused, global field pose. */
  private static final SwerveDrivePoseEstimator fieldLocalizer =
      new SwerveDrivePoseEstimator(
          kinematics,
          Rotation2d.kZero,
          new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
          },
          new Pose2d());

  /** Second estimator for reef-context vision fusion. */
  private static final SwerveDrivePoseEstimator reefLocalizer =
      new SwerveDrivePoseEstimator(
          kinematics,
          Rotation2d.kZero,
          new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
          },
          new Pose2d());

  /** Pure odometry (same inputs) for debugging/visibility. */
  private static final edu.wpi.first.math.kinematics.SwerveDriveOdometry odometry =
      new edu.wpi.first.math.kinematics.SwerveDriveOdometry(
          kinematics,
          Rotation2d.kZero,
          new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
          });

  // ===== Core state for heading anchoring and integration =====
  /** Latest raw gyro yaw provided by Drive (un-offset). */
  private static Rotation2d robotHeadingRaw = Rotation2d.kZero;

  /**
   * Heading offset so that (raw + offset) equals the estimator’s frame rotation. IMPORTANT: offset
   * = desiredRotation - rawGyroRotation
   */
  private static Rotation2d headingOffset = Rotation2d.kZero;

  /** Our current best yaw (raw + offset) used for estimator updates. */
  private static Rotation2d yawWithOffset = Rotation2d.kZero;

  /** Last wheel positions (for heading integration when gyro is missing for a cycle). */
  private static SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };

  /** Keep module positions for resets. Updated on every odometry observation. */
  private static SwerveModulePosition[] currentWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };

  // ===== Timestamped fused field pose buffer =====
  private static final double kPoseBufferSecs = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> fusedPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(kPoseBufferSecs);

  // ===== Speeds (measured & desired) =====
  private final AtomicReference<ChassisSpeeds> measuredRobotRelSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> measuredFieldRelSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredRobotRelSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredFieldRelSpeeds =
      new AtomicReference<>(new ChassisSpeeds());

  @Getter @Setter private static RobotMode mode = RobotMode.DISABLED;

  private RobotState() {
    fusedPoseBuffer.addSample(Timer.getTimestamp(), Pose2d.kZero);
  }

  // ============================================================
  // Odometry (Drive calls this with timestamped samples)
  // ============================================================

  /**
   * Add an odometry sample.
   *
   * @param timestampSeconds FPGA timestamp for this sample
   * @param wheelPositions module positions at this timestamp
   * @param gyroYawRaw optional raw gyro yaw at this timestamp (empty if unavailable)
   */
  public synchronized void addOdometryObservation(
      double timestampSeconds,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> gyroYawRaw) {

    currentWheelPositions = wheelPositions;

    if (gyroYawRaw.isPresent()) {
      robotHeadingRaw = gyroYawRaw.get();
      yawWithOffset = robotHeadingRaw.plus(headingOffset);
    } else {
      var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
      yawWithOffset = yawWithOffset.plus(new Rotation2d(twist.dtheta));
    }
    lastWheelPositions = wheelPositions;

    fieldLocalizer.updateWithTime(timestampSeconds, yawWithOffset, wheelPositions);
    reefLocalizer.updateWithTime(timestampSeconds, yawWithOffset, wheelPositions);
    odometry.update(yawWithOffset, wheelPositions);

    fusedPoseBuffer.addSample(timestampSeconds, fieldLocalizer.getEstimatedPosition());
  }

  // ============================================================
  // Vision
  // ============================================================

  public synchronized void addFieldVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    fieldLocalizer.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  public synchronized void addFieldVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, double xyStdDev) {
    fieldLocalizer.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY));
  }

  public synchronized void addReefVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    reefLocalizer.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  public synchronized void addReefVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, double xyStdDev) {
    reefLocalizer.addVisionMeasurement(
        visionPose,
        timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY));
  }

  // ============================================================
  // Speeds
  // ============================================================

  public void setMeasuredRobotRelativeSpeeds(ChassisSpeeds speeds) {
    measuredRobotRelSpeeds.set(speeds);
    measuredFieldRelSpeeds.set(
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRobotPoseField().getRotation()));
  }

  public void setMeasuredFieldRelativeSpeeds(ChassisSpeeds speeds) {
    measuredFieldRelSpeeds.set(speeds);
    measuredRobotRelSpeeds.set(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotPoseField().getRotation()));
  }

  public void setDesiredRobotRelativeSpeeds(ChassisSpeeds speeds) {
    desiredRobotRelSpeeds.set(speeds);
    desiredFieldRelSpeeds.set(
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRobotPoseField().getRotation()));
  }

  public void setDesiredFieldRelativeSpeeds(ChassisSpeeds speeds) {
    desiredFieldRelSpeeds.set(speeds);
    desiredRobotRelSpeeds.set(
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotPoseField().getRotation()));
  }

  public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
    return measuredRobotRelSpeeds.get();
  }

  public ChassisSpeeds getMeasuredFieldRelativeSpeeds() {
    return measuredFieldRelSpeeds.get();
  }

  public ChassisSpeeds getDesiredRobotRelativeSpeeds() {
    return desiredRobotRelSpeeds.get();
  }

  public ChassisSpeeds getDesiredFieldRelativeSpeeds() {
    return desiredFieldRelSpeeds.get();
  }

  // ============================================================
  // Resets / Getters / Prediction
  // ============================================================

  /** Reset all estimators to a pose; preserves current wheel positions and raw gyro reading. */
  public synchronized void resetRobotPose(Pose2d pose) {
    headingOffset = pose.getRotation().minus(robotHeadingRaw);
    yawWithOffset = robotHeadingRaw.plus(headingOffset);

    fieldLocalizer.resetPosition(yawWithOffset, currentWheelPositions, pose);
    reefLocalizer.resetPosition(yawWithOffset, currentWheelPositions, pose);
    odometry.resetPosition(yawWithOffset, currentWheelPositions, pose);

    fusedPoseBuffer.clear();
    fusedPoseBuffer.addSample(Timer.getTimestamp(), pose);
  }

  public Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  public Pose2d getRobotPoseReef() {
    return reefLocalizer.getEstimatedPosition();
  }

  public Pose2d getRobotPoseOdometry() {
    return odometry.getPoseMeters();
  }

  /** Get fused field pose at a prior timestamp. */
  public Optional<Pose2d> getPoseAt(double timestampSeconds) {
    return fusedPoseBuffer.getSample(timestampSeconds);
  }

  public Rotation2d getYawForVision() {
    return getRobotPoseField().getRotation();
  }

  /** Constant-velocity lookahead using measured robot-relative speeds. */
  public Pose2d getPredictedPose(double lookaheadSeconds) {
    Pose2d start = getRobotPoseField();
    ChassisSpeeds vRobot = measuredRobotRelSpeeds.get();
    var twist =
        new edu.wpi.first.math.geometry.Twist2d(
            vRobot.vxMetersPerSecond * lookaheadSeconds,
            vRobot.vyMetersPerSecond * lookaheadSeconds,
            vRobot.omegaRadiansPerSecond * lookaheadSeconds);
    return start.exp(twist);
  }

  // ============================================================
  // Types
  // ============================================================

  public enum RobotMode {
    DISABLED,
    TELEOP,
    AUTO;

    public static boolean enabled(RobotMode mode) {
      return mode == TELEOP || mode == AUTO;
    }

    public static boolean disabled(RobotMode mode) {
      return mode == DISABLED;
    }

    public static boolean teleop(RobotMode mode) {
      return mode == TELEOP;
    }

    public static boolean auto(RobotMode mode) {
      return mode == AUTO;
    }

    public static boolean enabled() {
      return enabled(RobotState.getMode());
    }

    public static boolean disabled() {
      return disabled(RobotState.getMode());
    }

    public static boolean teleop() {
      return teleop(RobotState.getMode());
    }

    public static boolean auto() {
      return auto(RobotState.getMode());
    }
  }
}
