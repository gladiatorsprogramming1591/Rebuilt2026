package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterSpeed = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * setting the shooter motor voltage
   *
   * @param speed voltage to set from -12 to 12
   */
  public default void setShooterSpeed(double speed) {}
}
