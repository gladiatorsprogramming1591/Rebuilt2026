package frc.robot.subsystems.hood;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the hood subsystem.
 *
 * <p>The subsystem owns hood behavior and writes desired outputs into {@link HoodIOOutputs}. Each IO
 * implementation reads hardware sensors into {@link HoodIOInputs} and applies the requested outputs
 * to real or simulated hardware.
 */
public interface HoodIO {
  /** Hood motor control mode. */
  enum HoodMode {
    /** Open-loop percent output. */
    SPEED,

    /** Closed-loop position control. */
    POSITION
  }

  /** Sensor values read from the hood hardware each loop. */
  @AutoLog
  class HoodIOInputs {
    /** Hood velocity in current mechanism units per second. */
    double hoodSpeed = 0.0;

    /** Hood position in current legacy hood units. */
    double hoodAngle = 0.0;

    /** Motor supply current in amps. */
    double hoodSupplyCurrent = 0.0;

    /** Motor stator current in amps. */
    double hoodStatorCurrent = 0.0;

    /** Motor torque current in amps. */
    double hoodTorqueCurrent = 0.0;

    /** Motor controller temperature in Celsius. */
    double hoodTemperature = 0.0;

    /** Raw DIO value from the bottom limit sensor. */
    boolean hoodLimitRaw = false;

    /** True when the bottom limit sensor is active/tripped. */
    boolean hoodLimitTripped = false;

    /**
     * Temporary compatibility field for older logging code.
     *
     * <p>Use {@link #hoodLimitTripped} for new code.
     */
    boolean hoodLimitSet = false;
  }

  /** Desired outputs written by the hood subsystem and applied by the IO implementation. */
  @AutoLog
  class HoodIOOutputs {
    /** True once the hood has accepted a mechanical zero since boot. */
    boolean hasBeenZeroed = false;

    /** Unused legacy debug field. */
    double positionRad = 0.0;

    /** Unused legacy debug field. */
    double velocityRadPerSecond = 0.0;

    /** Desired hood position in current legacy hood units. */
    double desiredHoodAngle = 0.0;

    /** Desired open-loop motor output. */
    double desiredHoodSpeed = 0.0;

    /** Desired hood control mode. Defaults to stopped open-loop output. */
    HoodMode mode = HoodMode.SPEED;

    /** Upward-motion PID proportional gain. */
    double upKP = 0.0;

    /** Upward-motion PID integral gain. */
    double upKI = 0.0;

    /** Upward-motion PID derivative gain. */
    double upKD = 0.0;

    /** Upward-motion PID static feedforward. */
    double upKS = 0.0;

    /** Downward-motion PID proportional gain. */
    double downKP = 0.0;

    /** Downward-motion PID integral gain. */
    double downKI = 0.0;

    /** Downward-motion PID derivative gain. */
    double downKD = 0.0;

    /** Downward-motion PID static feedforward. */
    double downKS = 0.0;
  }

  /**
   * Updates the latest hood sensor inputs.
   *
   * @param inputs container updated with the latest hood hardware state
   */
  default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Applies the latest desired hood outputs.
   *
   * @param outputs latest requested hood outputs from the subsystem
   */
  default void applyOutputs(HoodIOOutputs outputs) {}

  /**
   * Optional connection check for hardware implementations.
   *
   * @param inputs latest hood inputs
   * @return true if the hood hardware is connected
   */
  default boolean getIsConnected(HoodIOInputs inputs) {
    return false;
  }

  /**
   * Seeds the hood hardware encoder to zero.
   *
   * <p>This does not move the mechanism. The subsystem should only call this after the hood has
   * reached the zero hard stop or after a trusted manual zero command.
   */
  default void zeroHood() {}

  /**
   * Returns whether the hood encoder is close to zero.
   *
   * <p>This checks encoder position only. It does not prove the hood is physically touching the zero
   * hard stop.
   *
   * @return supplier that is true when the hood encoder is within the configured zero tolerance
   */
  default BooleanSupplier isHoodWithinZeroTolerance() {
    return () -> false;
  }

  /**
   * Returns whether the hood is physically at true zero.
   *
   * <p>Hardware implementations may use a bottom limit sensor, stall detection, or both.
   *
   * @return true when the hood should be considered physically at the zero hard stop
   */
  default boolean isHoodAtTrueZero() {
    return false;
  }

  /**
   * Returns whether the hood appears stopped or stalled for a minimum duration.
   *
   * @param minStationaryDuration minimum duration in seconds
   * @return true when the hood has been stopped long enough
   */
  default boolean hasHoodStoppedOverTime(double minStationaryDuration) {
    return false;
  }

  /** Resets the timer used by the zeroing fallback. */
  default void resetHoodTimer() {}

  /**
   * Sets the hood motor supply current limit.
   *
   * @param supplyLimitAmps supply current limit in amps
   */
  default void setSupplyCurrentLimit(double supplyLimitAmps) {}

  /**
   * Sets the hood motor stator current limit.
   *
   * @param statorLimitAmps stator current limit in amps
   */
  default void setStatorCurrentLimit(double statorLimitAmps) {}
}