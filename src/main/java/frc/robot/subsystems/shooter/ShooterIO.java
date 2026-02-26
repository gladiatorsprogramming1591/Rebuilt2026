package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocity = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * setting the shooter motor voltage
   *
   * @param shooterVelocity voltage to set from -12 to 12
   */
  public default void runShooter(double shooterVelocity) {}
}
