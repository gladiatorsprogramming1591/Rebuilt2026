package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    double hoodSpeed = 0.0;
  }

  /**
   * Refreshes the {@link HoodIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(HoodIOInputs inputs) {}
  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(HoodIOInputs inputs) {
    return false;
  }

  /**
   * Set the motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  public default void setHoodSpeed(double speed) {}
}
