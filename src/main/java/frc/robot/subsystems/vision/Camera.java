package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import frc.robot.util.LoggedTracer;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Thin wrapper around a {@link CameraIO} implementation that holds static config and exposes
 * per-loop inputs via AdvantageKit.
 */
public class Camera {
  private final CameraIOInputsAutoLogged inputs;

  private final CameraIO io;
  @Getter private final Pose3d cameraOffset;
  @Getter private final String name;
  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;
  @Getter private final DoubleArrayPublisher robotHeadingPublisher;
  @Getter private final List<CameraDuty> cameraDuties;
  @Getter private Vision.VisionEstimationMode visionMode = Vision.VisionEstimationMode.MT2;

  /**
   * Constructs a camera with a fixed transform and duties.
   *
   * @param io hardware IO implementation
   * @param horizontalFOV horizontal field of view (radians)
   * @param verticalFOV vertical field of view (radians)
   * @param primaryXYStandardDeviationCoefficient std-dev model coefficient for primary pose
   * @param secondaryXYStandardDeviationCoefficient std-dev model coefficient for secondary pose
   * @param robotHeadingPublisher NT publisher for robot yaw (used by MT2)
   * @param cameraDuties functional roles for this camera
   * @param offset camera-to-robot transform
   */
  public Camera(
      CameraIO io,
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient,
      DoubleArrayPublisher robotHeadingPublisher,
      List<CameraDuty> cameraDuties,
      Transform3d offset) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    io.setCameraOffset(offset);
    this.cameraOffset = new Pose3d(offset.getTranslation(), offset.getRotation());
    this.name = io.getName();
    this.cameraType = io.getCameraType();
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
    this.robotHeadingPublisher = robotHeadingPublisher;
    this.cameraDuties = cameraDuties;
  }

  /**
   * Constructs a camera without an explicit transform (defaults to identity).
   *
   * @param io hardware IO implementation
   * @param name display name
   * @param horizontalFOV horizontal field of view (radians)
   * @param verticalFOV vertical field of view (radians)
   * @param primaryXYStandardDeviationCoefficient std-dev model coefficient for primary pose
   * @param secondaryXYStandardDeviationCoefficient std-dev model coefficient for secondary pose
   * @param robotHeadingPublisher NT publisher for robot yaw (used by MT2)
   * @param cameraDuties functional roles for this camera
   */
  public Camera(
      CameraIO io,
      String name,
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient,
      DoubleArrayPublisher robotHeadingPublisher,
      List<CameraDuty> cameraDuties) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    this.cameraOffset = new Pose3d();
    this.name = name;
    this.cameraType = io.getCameraType();
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
    this.robotHeadingPublisher = robotHeadingPublisher;
    this.cameraDuties = cameraDuties;
  }

  /** Pulls fresh inputs from the IO and logs them. Call once per loop. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + name, inputs);
    Logger.recordOutput("Vision/Camera/" + name + "/Mode", visionMode.name());
    LoggedTracer.record("Camera/" + name);
  }

  /**
   * Sets the per-camera processing mode (MT1/MT2/SingleTagGyro/TxTyTa).
   *
   * @param mode mode to use for this camera
   */
  // TODO: Determine how we want to set this vision mode
  public void setVisionMode(Vision.VisionEstimationMode mode) {
    this.visionMode = mode;
  }

  /**
   * @return latest tx wrapped as {@link Rotation2d}
   */
  public Rotation2d getXOffset() {
    return inputs.data.xOffset();
  }

  /**
   * @return latest ty wrapped as {@link Rotation2d}
   */
  public Rotation2d getYOffset() {
    return inputs.data.yOffset();
  }

  /**
   * @return whether a target is currently detected
   */
  public boolean getTargetAquired() {
    return inputs.data.targetAquired();
  }

  /**
   * @return number of detected targets
   */
  public int getTotalTargets() {
    return inputs.data.totalTargets();
  }

  /**
   * @return average measured distance to tags (meters)
   */
  public double getAverageDistance() {
    return inputs.data.averageDistance();
  }

  /**
   * @return capture timestamp (seconds)
   */
  public double getFrameTimestamp() {
    return inputs.data.frameTimestamp();
  }

  /**
   * @return primary pose estimate from the IO (e.g., MT2)
   */
  public Pose2d getPrimaryPose() {
    return inputs.data.primaryPose();
  }

  /**
   * @return secondary pose estimate from the IO (e.g., MT1)
   */
  public Pose2d getSecondaryPose() {
    return inputs.data.secondaryPose();
  }

  /**
   * @return last seen fiducial ID, or -1 if unknown
   */
  public double getTagIDOfInterest() {
    return inputs.data.tagIDOfInterest();
  }

  /**
   * @return true if the IO reports camera is connected
   */
  public boolean getIsConnected() {
    return inputs.data.isConnected();
  }

  /**
   * Sets the device pipeline.
   *
   * @param pipeline pipeline index
   */
  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
  }

  /**
   * Restricts the set of valid AprilTag IDs used by the device for localization.
   *
   * @param validIds list of allowed IDs
   */
  public void setValidTags(int... validIds) {
    io.setValidTags(validIds);
  }

  /**
   * @return underlying IO implementation (for advanced control)
   */
  public CameraIO getIo() {
    return io;
  }
}
