package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.AngularVelocity;
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
    double hoodTorqueCurrent = 0.0;
    double hoodTemperature = 0.0;
    boolean hoodLimitSet = false;
  }

  public static class HoodIOOutputs {
    boolean hasBeenZeroed = false;
    double positionRad = 0.0;
    double velocityRadPerSecond = 0.0;
    double desiredHoodAngle;
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

  /**
   * Set the motor speed to {@code speed}.
   *
   * @param speed speed to set motor to
   */
  public default void setHoodSpeed(AngularVelocity angularVelocity) {}

  public default void setHoodSpeed(double speed) {}

  public default void setHoodPosition(double angle) {}

  public default void setHoodCurrentLimit(double currentLimit) {}

  public default void zero() {}

  public default void driveToZero() {}

  public default boolean isHoodStationary(boolean withTimer) {
    return false;
  }

  public default boolean isHoodAtTrueZero() {
    return false;
  }
}
