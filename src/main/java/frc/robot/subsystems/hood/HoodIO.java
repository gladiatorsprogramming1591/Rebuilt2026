package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    double hoodSpeed = 0.0;
    double hoodAngle = 0.0; 
<<<<<<< HEAD
    double hoodSupplyCurrent = 0.0;
    double hoodTorqueCurrent = 0.0;
    double hoodTemperature = 0.0;
=======
>>>>>>> origin/KileyHood
  }

  public static class HoodIOOutputs{
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
<<<<<<< HEAD
  public default void setHoodSpeed(AngularVelocity angularVelocity) {}

  public default void setHoodPosition(double angle) {}

  public default void zeroHoodEncoder() {}
=======
  public default void setHoodSpeed(double speed) {}

  public default void setHoodPosition(double angle) {}
>>>>>>> origin/KileyHood
}
