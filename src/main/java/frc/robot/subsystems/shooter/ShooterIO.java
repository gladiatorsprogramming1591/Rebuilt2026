package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterSpeed = 0.0;
    public double shooterVelocity = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * setting the shooter motor voltage
   *
   * @param speed voltage to set from -12 to 12
   */
  public default void runShooter(double speed) {}

  /**
   * setting the shooter motor velocity
   *
   * @param shooterVelocity velocity to set in ticks per 100ms
   */
  public default void runShooterTarget(double shooterVelocity) {}
}
