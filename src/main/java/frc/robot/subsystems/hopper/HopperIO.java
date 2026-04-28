package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the hopper subsystem.
 *
 * <p>The hopper subsystem owns belt behavior and writes desired outputs into {@link
 * HopperIOOutputs}. Each IO implementation reads hardware sensors into {@link HopperIOInputs} and
 * applies the requested outputs to real or simulated hardware.
 */
public interface HopperIO {
  /** Sensor values read from the hopper hardware each loop. */
  @AutoLog
  class HopperIOInputs {
    /** Belt motor supply current in amps. */
    double beltCurrent = 0.0;

    /** Belt motor velocity in motor rotations per second. */
    double beltVelocity = 0.0;

    /** True when the hopper empty sensor is connected and reporting a valid empty condition. */
    boolean hopperEmpty = false;

    /** True when the CANrange hopper empty sensor is connected. */
    boolean hopperEmptySensorConnected = false;

    /** Distance measured by the hopper empty CANrange in meters. */
    double hopperEmptyDistance = 1.0;
  }

  /** Desired outputs written by the hopper subsystem and applied by the IO implementation. */
  @AutoLog
  class HopperIOOutputs {
    /** Desired open-loop belt motor output. */
    double beltSpeed = 0.0;

    /**
     * True when the belt should use the lower current limit intended for intaking/agitation.
     *
     * <p>Reverse and normal shooting belt commands should leave this false so the full current
     * limit is used.
     */
    boolean useBeltWhileIntakeCurrent = false;
  }

  /**
   * Updates the latest hopper sensor inputs.
   *
   * @param inputs container updated with the latest hopper hardware state
   */
  default void updateInputs(HopperIOInputs inputs) {}

  /**
   * Applies the latest desired hopper outputs.
   *
   * @param outputs latest requested hopper outputs from the subsystem
   */
  default void applyOutputs(HopperIOOutputs outputs) {}

  /**
   * Optional connection check for hardware implementations.
   *
   * @param inputs latest hopper inputs
   * @return true if the hopper hardware is connected
   */
  default boolean getIsConnected(HopperIOInputs inputs) {
    return false;
  }
}
