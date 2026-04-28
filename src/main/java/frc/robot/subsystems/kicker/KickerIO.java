package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the kicker subsystem.
 *
 * <p>The kicker subsystem owns behavior and writes desired outputs into {@link KickerIOOutputs}.
 * Each IO implementation reads hardware sensors into {@link KickerIOInputs} and applies the
 * requested outputs to real or simulated hardware.
 */
public interface KickerIO {
  /** Sensor values read from kicker hardware each loop. */
  @AutoLog
  class KickerIOInputs {
    /** True when the primary kicker motor is connected. */
    boolean primaryConnected = false;

    /** Primary kicker motor duty cycle output. */
    double primaryDutyCycle = 0.0;

    /** Primary kicker motor velocity in motor rotations per second. */
    double primaryVelocity = 0.0;

    /** Primary kicker motor applied voltage. */
    double primaryAppliedVolts = 0.0;

    /** Primary kicker motor supply current in amps. */
    double primarySupplyCurrent = 0.0;

    /** Primary kicker motor stator current in amps. */
    double primaryStatorCurrent = 0.0;

    /** Primary kicker motor torque current in amps. */
    double primaryTorqueCurrent = 0.0;

    /** Primary kicker motor temperature in Celsius. */
    double primaryTemperature = 0.0;

    /** True when the secondary kicker motor is present and connected. */
    boolean secondaryConnected = false;

    /** Secondary kicker motor duty cycle output. */
    double secondaryDutyCycle = 0.0;

    /** Secondary kicker motor velocity in motor rotations per second. */
    double secondaryVelocity = 0.0;

    /** Secondary kicker motor applied voltage. */
    double secondaryAppliedVolts = 0.0;

    /** Secondary kicker motor supply current in amps. */
    double secondarySupplyCurrent = 0.0;

    /** Secondary kicker motor stator current in amps. */
    double secondaryStatorCurrent = 0.0;

    /** Secondary kicker motor torque current in amps. */
    double secondaryTorqueCurrent = 0.0;

    /** Secondary kicker motor temperature in Celsius. */
    double secondaryTemperature = 0.0;
  }

  /** Desired outputs written by the kicker subsystem and applied by the IO implementation. */
  @AutoLog
  class KickerIOOutputs {
    /** Desired open-loop kicker motor output. */
    double desiredKickerSpeed = 0.0;
  }

  /**
   * Updates the latest kicker sensor inputs.
   *
   * @param inputs container updated with the latest kicker hardware state
   */
  default void updateInputs(KickerIOInputs inputs) {}

  /**
   * Applies the latest desired kicker outputs.
   *
   * @param outputs latest requested kicker outputs from the subsystem
   */
  default void applyOutputs(KickerIOOutputs outputs) {}

  /**
   * Optional connection check for hardware implementations.
   *
   * @param inputs latest kicker inputs
   * @return true if the primary kicker hardware is connected
   */
  default boolean getIsConnected(KickerIOInputs inputs) {
    return inputs.primaryConnected;
  }
}
