package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Lightweight timing helper for finding loop overruns without spamming logs every robot loop. */
public final class LoopProfiler {
  private static final String TABLE_KEY = "Profiling/";

  private static final LoggedTunableBoolean enabled =
      new LoggedTunableBoolean(TABLE_KEY + "Enabled", true, Constants.tuningMode);

  private static final LoggedTunableNumber periodLoops =
      new LoggedTunableNumber(TABLE_KEY + "PeriodLoops", 25.0, Constants.tuningMode);

  private static final Map<String, Stats> statsByName = new HashMap<>();
  private static int loopCounter = 0;

  private LoopProfiler() {}

  /** Runs a profiled block. When profiling is disabled this only runs the block. */
  public static void run(String name, Runnable block) {
    if (!isEnabled()) {
      block.run();
      return;
    }

    double startTimestamp = Timer.getFPGATimestamp();
    try {
      block.run();
    } finally {
      record(name, Timer.getFPGATimestamp() - startTimestamp);
    }
  }

  /** Runs a profiled supplier and returns its result. */
  public static <T> T get(String name, Supplier<T> block) {
    if (!isEnabled()) {
      return block.get();
    }

    double startTimestamp = Timer.getFPGATimestamp();
    try {
      return block.get();
    } finally {
      record(name, Timer.getFPGATimestamp() - startTimestamp);
    }
  }

  /** Publishes timing stats at the configured slow cadence. Call once from robotPeriodic. */
  public static void periodic() {
    if (!isEnabled()) {
      statsByName.clear();
      loopCounter = 0;
      return;
    }

    loopCounter++;
    int requestedPeriodLoops = Math.max(1, (int) Math.round(periodLoops.getAsDouble()));
    if (loopCounter < requestedPeriodLoops) {
      return;
    }

    loopCounter = 0;
    Logger.recordOutput(TABLE_KEY + "PeriodLoopsActive", requestedPeriodLoops);

    for (Map.Entry<String, Stats> entry : statsByName.entrySet()) {
      Stats stats = entry.getValue();
      String key = TABLE_KEY + entry.getKey() + "/";
      Logger.recordOutput(key + "LatestMs", stats.latestSeconds * 1000.0);
      Logger.recordOutput(key + "AverageMs", stats.getAverageSeconds() * 1000.0);
      Logger.recordOutput(key + "MaxMs", stats.maxSeconds * 1000.0);
      Logger.recordOutput(key + "Samples", stats.samples);
      stats.resetWindow();
    }
  }

  private static boolean isEnabled() {
    return Constants.tuningMode && enabled.getAsBoolean();
  }

  private static void record(String name, double seconds) {
    statsByName.computeIfAbsent(name, ignored -> new Stats()).record(seconds);
  }

  private static final class Stats {
    private double latestSeconds = 0.0;
    private double totalSeconds = 0.0;
    private double maxSeconds = 0.0;
    private int samples = 0;

    private void record(double seconds) {
      latestSeconds = seconds;
      totalSeconds += seconds;
      maxSeconds = Math.max(maxSeconds, seconds);
      samples++;
    }

    private double getAverageSeconds() {
      return samples == 0 ? 0.0 : totalSeconds / samples;
    }

    private void resetWindow() {
      totalSeconds = 0.0;
      maxSeconds = 0.0;
      samples = 0;
    }
  }
}
