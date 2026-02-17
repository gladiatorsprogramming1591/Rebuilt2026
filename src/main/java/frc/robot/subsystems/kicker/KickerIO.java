package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    double kickerSpeed = 0.0;
  }

  /**
   * Refreshes the {@link KickerIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(KickerIOInputs inputs) {}
  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(KickerIOInputs inputs) {
    return false;
  }

  /**
   * Set the motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  public default void setKickerSpeed(double speed) {}
}

