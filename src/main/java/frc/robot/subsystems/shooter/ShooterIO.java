package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double RL_RPS;
    public double RF_RPS;
    public double LL_RPS;
    public double LF_RPS;
    public double RL_appliedVolts;
    public double RF_appliedVolts;
    public double LL_appliedVolts;
    public double LF_appliedVolts;
    public double RL_motorTemp;
    public double RF_motorTemp;
    public double LL_motorTemp;
    public double LF_motorTemp;
    public double RL_supplyCurrent;
    public double RF_supplyCurrent;
    public double LL_supplyCurrent;
    public double LF_supplyCurrent;
    public double RL_torqueCurrentAmps;
    public double RF_torqueCurrentAmps;
    public double LL_torqueCurrentAmps;
    public double LF_torqueCurrentAmps;
    public boolean shooterAtVelocity;
  }

  @AutoLog
  public class ShooterIOOutputs {
    public double desiredVelocityRPM = 0.0;
    public double desiredDutyCycle = 0.0;
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public double kMMAcceleration;
    public double kMMJerk;
    public boolean useMotionMagic = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}

  /**
   * setting the shooter motor voltage
   *
   * @param voltage voltage to set from -12 to 12
   */
  public default void runShooterVoltage(double voltage) {}

  public default void runShooterDutyCycle(double shooterVelocity) {}

  /**
   * Checks if the velocity of the right leader motor is within tolerance of the target velocity.
   * <p>
   * IOKraken: Only works with positive (i.e. shooting) velocity targets, not negative.
   *
   * @return Boolean supplier of whether right leader motor velocity is at target
   */
  public default BooleanSupplier rightShooterAtVelocity() {
    return () -> false;
  }

  /**
   * Checks if the velocity of the right leader motor is within tolerance of the target velocity.
   * <p>
   * IOKraken: Only works with positive (i.e. shooting) velocity targets, not negative.
   *
   * @param targetRPS
   * @return Boolean supplier of whether right leader motor velocity is at target
   */
  public default BooleanSupplier rightShooterAtVelocity(DoubleSupplier targetRPS) {
    return () -> false;
  }

  /**
   * Checks if the velocity of the right leader motor is below coast RPM + tolerance.
   *
   * @return Boolean supplier of whether right leader motor velocity is below coast RPM
   */
  public default BooleanSupplier rightShooterBelowCoastRPM() {
    return () -> false;
  }

  public default BooleanSupplier leftShooterAtVelocity() {
    return () -> false;
  }

  public default BooleanSupplier bothShootersAtVelocity() {
    return () -> false;
  }

  public default void setShooterMotorRPM(double rpm) {}

  /**
   * Applies the latest tunable TalonFX configurations to <b>all shooter motors</b>.
   * <p>
   * Only applies the configuration when the Smartdashboard boolean {@value #UPDATE_CONFIG_NAME} is changed from false to true (i.e. rising edge).
   * 
   * @param outputs Shooter outputs where the tunable configurations are accessable
   * @see {@link #createTunedMotorConfig(frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs) createTunedMotorConfig()}
   * @see frc.robot.util.LoggedTunableNumber LoggedTunableNumber
   */
  public default void tuneMotorConfigs(ShooterIOOutputs outputs) {}
}
