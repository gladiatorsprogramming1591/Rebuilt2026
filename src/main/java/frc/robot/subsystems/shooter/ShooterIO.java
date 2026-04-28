package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the shooter subsystem.
 *
 * <p>The shooter subsystem owns behavior and writes desired outputs into {@link ShooterIOOutputs}.
 * Each IO implementation reads hardware sensors into {@link ShooterIOInputs} and applies the
 * requested outputs to real or simulated hardware.
 */
public interface ShooterIO {
  /** Sensor values read from shooter hardware each loop. */
  @AutoLog
  class ShooterIOInputs {
    public boolean rightLeaderConnected = false;
    public boolean rightFollowerConnected = false;
    public boolean leftLeaderConnected = false;
    public boolean leftFollowerConnected = false;

    public double rightLeaderVelocityRPM = 0.0;
    public double rightFollowerVelocityRPM = 0.0;
    public double leftLeaderVelocityRPM = 0.0;
    public double leftFollowerVelocityRPM = 0.0;

    public double rightLeaderAppliedVolts = 0.0;
    public double rightFollowerAppliedVolts = 0.0;
    public double leftLeaderAppliedVolts = 0.0;
    public double leftFollowerAppliedVolts = 0.0;

    public double rightLeaderTemperature = 0.0;
    public double rightFollowerTemperature = 0.0;
    public double leftLeaderTemperature = 0.0;
    public double leftFollowerTemperature = 0.0;

    public double rightLeaderSupplyCurrent = 0.0;
    public double rightFollowerSupplyCurrent = 0.0;
    public double leftLeaderSupplyCurrent = 0.0;
    public double leftFollowerSupplyCurrent = 0.0;

    public double rightLeaderStatorCurrent = 0.0;
    public double rightFollowerStatorCurrent = 0.0;
    public double leftLeaderStatorCurrent = 0.0;
    public double leftFollowerStatorCurrent = 0.0;

    public double rightLeaderTorqueCurrent = 0.0;
    public double rightFollowerTorqueCurrent = 0.0;
    public double leftLeaderTorqueCurrent = 0.0;
    public double leftFollowerTorqueCurrent = 0.0;

    /** True when the right leader is within tolerance of the current target. */
    public boolean shooterAtVelocity = false;
  }

  /** Desired outputs written by the shooter subsystem and applied by the IO implementation. */
  @AutoLog
  class ShooterIOOutputs {
    public double desiredVelocityRPM = 0.0;
    public double desiredDutyCycle = 0.0;

    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double kMMAcceleration = 0.0;
    public double kMMJerk = 0.0;

    public boolean useMotionMagic = false;
  }

  /**
   * Updates the latest shooter sensor inputs.
   *
   * @param inputs container updated with the latest shooter hardware state
   */
  default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Applies the latest desired shooter outputs.
   *
   * @param outputs latest requested shooter outputs from the subsystem
   */
  default void applyOutputs(ShooterIOOutputs outputs) {}

  /**
   * Returns whether the right leader is at the most recently commanded shooter target.
   *
   * @return true when the right leader is within the configured RPM tolerance
   */
  default BooleanSupplier rightShooterAtVelocity() {
    return () -> false;
  }

  /**
   * Returns whether the right leader is at a provided RPM target.
   *
   * @param targetRPM target flywheel velocity in RPM
   * @return true when the right leader is within the configured RPM tolerance
   */
  default BooleanSupplier rightShooterAtVelocityRPM(DoubleSupplier targetRPM) {
    return () -> false;
  }

  /**
   * Returns whether the right leader is below the configured coast RPM plus tolerance.
   *
   * @return true when the shooter has slowed below coast speed
   */
  default BooleanSupplier rightShooterBelowCoastRPM() {
    return () -> false;
  }

  /**
   * Applies the latest tunable motor configuration values.
   *
   * @param outputs latest requested shooter outputs containing tunable values
   */
  default void tuneMotorConfigs(ShooterIOOutputs outputs) {}
}