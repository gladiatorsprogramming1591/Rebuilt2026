package frc.robot.subsystems.hood;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  public enum HoodMode {
    SPEED,
    POSITION;
  }

  @AutoLog
  public static class HoodIOInputs {
    double hoodSpeed = 0.0;
    double hoodAngle = 0.0;
    double hoodSupplyCurrent = 0.0;
    double hoodStatorCurrent = 0.0;
    double hoodTorqueCurrent = 0.0;
    double hoodTemperature = 0.0;
    boolean hoodLimitSet = false;
  }

  @AutoLog
  public static class HoodIOOutputs {
    boolean hasBeenZeroed = false;
    double positionRad = 0.0;
    double velocityRadPerSecond = 0.0;
    double desiredHoodAngle = 0.0;
    double desiredHoodSpeed = 0.0;
    HoodMode mode;
    double kP = 0.0;
    double kD = 0.0;
    double kS = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}

  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(HoodIOInputs inputs) {
    return false;
  }

  public default void setHoodPosition(double angle) {}

  /**
   * Sets a supply current limit to the hood motor.
   * 
   * @param supplyLimitAmps Supply current limit in Amps to set to the motor.
   */
  public default void setSupplyCurrentLimit(double supplyLimitAmps) {}

  /**
   * Sets a stator current limit to the hood motor.
   * 
   * @param statorLimitAmps Stator current limit in Amps to set to the motor.
   */
  public default void setStatorCurrentLimit(double statorLimitAmps) {}

  public default void zeroHood() {}

  public default void runHoodToZero() {}

  public default BooleanSupplier isHoodWithinZeroTolerance() {
    return () -> false;
  }

  public default boolean hasHoodStoppedOverTime(double minStationaryDuration) {
    return false;
  }

  public default void resetHoodTimer() {}

  public default boolean isHoodAtTrueZero() {
    return false;
  }
}
