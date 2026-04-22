package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    double beltCurrent = 0.0;
    boolean hopperEmpty = false;
    double hopperEmptyDistance = 1.0; // Iniitalized to some value beyond the far hopper wall
  }

  @AutoLog
  public static class HopperIOOutputs {
    double beltSpeed = 0.0;
    boolean useBeltWhileIntakeCurrent = false;
    boolean usingLowerCurrent = false;
  }

  /**
   * Refreshes the {@link HopperIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(HopperIOInputs inputs) {}

  public default void applyOutputs(HopperIOOutputs outputs) {}

  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(HopperIOInputs inputs) {
    return false;
  }

  /**
   * Set the top motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  // public default void setTopRollerSpeed(double speed) {}

  /**
   * Set the bottom motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  // public default void setBottomRollerSpeed(double speed) {}
}
