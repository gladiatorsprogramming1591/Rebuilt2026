package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/** Switches Limelight pipelines for HDR-style sun handling outside the main vision loop. */
public class AutoHDRPipelineSwitcher {
  private static final String AUTO_HDR_ENABLED_KEY = "Vision/AutoHDR/Enabled";
  private static final String AUTO_HDR_NORMAL_PIPELINE_KEY = "Vision/AutoHDR/Normal Pipeline";
  private static final String AUTO_HDR_SUN_PIPELINE_KEY = "Vision/AutoHDR/Sun Pipeline";
  private static final String AUTO_HDR_SWITCH_SECONDS_KEY = "Vision/AutoHDR/Switch Seconds";

  private static final int DEFAULT_AUTO_HDR_NORMAL_PIPELINE = 0;
  private static final int DEFAULT_AUTO_HDR_SUN_PIPELINE = 1;
  private static final double DEFAULT_AUTO_HDR_SWITCH_SECONDS = 0.25;
  private static final double MIN_AUTO_HDR_SWITCH_SECONDS = 0.25;

  private static final int MIN_LIMELIGHT_PIPELINE = 0;
  private static final int MAX_LIMELIGHT_PIPELINE = 9;
  private static final int MAX_AUTO_HDR_CAMERAS = 2;

  private final Object stateLock = new Object();

  private final Camera[] cameras;
  private final int[] lastAppliedPipelineByCamera;
  private final int[] desiredPipelineByCamera;
  private final Notifier notifier;

  private boolean autoHdrWasEnabled = false;
  private int autoHdrPhase = 0;

  private boolean loggedEnabled = false;
  private int loggedPhase = 0;
  private int loggedCamera0Pipeline = -1;
  private int loggedCamera1Pipeline = -1;

  public AutoHDRPipelineSwitcher(Camera[] cameras) {
    this.cameras = cameras;
    this.lastAppliedPipelineByCamera = new int[cameras.length];
    this.desiredPipelineByCamera = new int[cameras.length];

    Arrays.fill(this.lastAppliedPipelineByCamera, -1);
    Arrays.fill(this.desiredPipelineByCamera, -1);

    SmartDashboard.setDefaultBoolean(AUTO_HDR_ENABLED_KEY, false);
    SmartDashboard.setDefaultNumber(AUTO_HDR_NORMAL_PIPELINE_KEY, DEFAULT_AUTO_HDR_NORMAL_PIPELINE);
    SmartDashboard.setDefaultNumber(AUTO_HDR_SUN_PIPELINE_KEY, DEFAULT_AUTO_HDR_SUN_PIPELINE);
    SmartDashboard.setDefaultNumber(AUTO_HDR_SWITCH_SECONDS_KEY, DEFAULT_AUTO_HDR_SWITCH_SECONDS);

    notifier = new Notifier(this::runAndReschedule);
    notifier.setName("AutoHDRPipelineSwitcher");
    notifier.startSingle(getSwitchSeconds());
  }

  /** Called from Vision.periodic() so camera writes and AdvantageKit logging stay on the main robot loop. */
  public void periodic() {
    int cameraCount = Math.min(MAX_AUTO_HDR_CAMERAS, cameras.length);
    int[] pipelinesToApply = new int[cameraCount];

    boolean enabled;
    int phase;
    int camera0Pipeline;
    int camera1Pipeline;

    synchronized (stateLock) {
      for (int i = 0; i < cameraCount; i++) {
        pipelinesToApply[i] = desiredPipelineByCamera[i];
      }

      enabled = loggedEnabled;
      phase = loggedPhase;
      camera0Pipeline = loggedCamera0Pipeline;
      camera1Pipeline = loggedCamera1Pipeline;
    }

    for (int i = 0; i < cameraCount; i++) {
      applyPipelineIfChanged(i, pipelinesToApply[i]);
    }

    if (!enabled && !Constants.Tuning.VISION) {
      return;
    }

    Logger.recordOutput("Vision/AutoHDR/Enabled", enabled);
    Logger.recordOutput("Vision/AutoHDR/CalibrationModeDisabled", Constants.calibrationMode);
    Logger.recordOutput("Vision/AutoHDR/Phase", phase);
    Logger.recordOutput("Vision/AutoHDR/Camera0Pipeline", camera0Pipeline);
    if (cameras.length > 1) {
      Logger.recordOutput("Vision/AutoHDR/Camera1Pipeline", camera1Pipeline);
    }
  }

  private void runAndReschedule() {
    try {
      updateAutoHdrPipelineSwitching();
    } finally {
      notifier.startSingle(getSwitchSeconds());
    }
  }

  private void updateAutoHdrPipelineSwitching() {
    if (cameras.length == 0) {
      synchronized (stateLock) {
        loggedEnabled = false;
        loggedPhase = 0;
        loggedCamera0Pipeline = -1;
        loggedCamera1Pipeline = -1;
      }
      return;
    }

    boolean enabled =
        !Constants.calibrationMode && SmartDashboard.getBoolean(AUTO_HDR_ENABLED_KEY, false);
    int normalPipeline =
        sanitizePipeline(
            (int)
                SmartDashboard.getNumber(
                    AUTO_HDR_NORMAL_PIPELINE_KEY, DEFAULT_AUTO_HDR_NORMAL_PIPELINE));
    int sunPipeline =
        sanitizePipeline(
            (int) SmartDashboard.getNumber(AUTO_HDR_SUN_PIPELINE_KEY, DEFAULT_AUTO_HDR_SUN_PIPELINE));

    if (!enabled) {
      synchronized (stateLock) {
        if (autoHdrWasEnabled) {
          for (int i = 0; i < Math.min(MAX_AUTO_HDR_CAMERAS, cameras.length); i++) {
            desiredPipelineByCamera[i] = normalPipeline;
          }
        }

        autoHdrWasEnabled = false;
        autoHdrPhase = 0;

        loggedEnabled = false;
        loggedPhase = 0;
        loggedCamera0Pipeline = cameras.length > 0 ? desiredPipelineByCamera[0] : -1;
        loggedCamera1Pipeline = cameras.length > 1 ? desiredPipelineByCamera[1] : -1;
      }
      return;
    }

    int firstPipeline = normalPipeline;
    int secondPipeline = sunPipeline;

    switch (autoHdrPhase) {
      case 0:
        firstPipeline = normalPipeline;
        secondPipeline = sunPipeline;
        break;
      case 1:
        firstPipeline = normalPipeline;
        secondPipeline = normalPipeline;
        break;
      case 2:
        firstPipeline = sunPipeline;
        secondPipeline = normalPipeline;
        break;
      default:
        firstPipeline = sunPipeline;
        secondPipeline = sunPipeline;
        break;
    }

    synchronized (stateLock) {
      autoHdrWasEnabled = true;

      desiredPipelineByCamera[0] = firstPipeline;
      if (cameras.length > 1) {
        desiredPipelineByCamera[1] = secondPipeline;
      }

      loggedEnabled = true;
      loggedPhase = autoHdrPhase;
      loggedCamera0Pipeline = firstPipeline;
      loggedCamera1Pipeline = cameras.length > 1 ? secondPipeline : -1;

      autoHdrPhase = (autoHdrPhase + 1) % 4;
    }
  }

  private double getSwitchSeconds() {
    double switchSeconds =
        SmartDashboard.getNumber(AUTO_HDR_SWITCH_SECONDS_KEY, DEFAULT_AUTO_HDR_SWITCH_SECONDS);
    if (!Double.isFinite(switchSeconds)) {
      switchSeconds = DEFAULT_AUTO_HDR_SWITCH_SECONDS;
    }
    return Math.max(MIN_AUTO_HDR_SWITCH_SECONDS, switchSeconds);
  }

  private int sanitizePipeline(int pipeline) {
    return Math.max(MIN_LIMELIGHT_PIPELINE, Math.min(MAX_LIMELIGHT_PIPELINE, pipeline));
  }

  private void applyPipelineIfChanged(int cameraIndex, int pipeline) {
    if (cameraIndex < 0 || cameraIndex >= cameras.length) {
      return;
    }

    if (pipeline < 0) {
      return;
    }

    if (lastAppliedPipelineByCamera[cameraIndex] == pipeline) {
      return;
    }

    cameras[cameraIndex].setPipeline(pipeline);
    lastAppliedPipelineByCamera[cameraIndex] = pipeline;
  }
}