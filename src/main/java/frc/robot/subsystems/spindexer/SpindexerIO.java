package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    double speed = 0.0;
  }

  /**
   * Refreshes the {@link SpindexerIOInputs} object with the latest sensor readings and derived
   * values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(SpindexerIOInputs inputs) {}
  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(SpindexerIOInputs inputs) {
    return false;
  }

  /**
   * Set the motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  public default void setSpeed(double speed) {}
}
