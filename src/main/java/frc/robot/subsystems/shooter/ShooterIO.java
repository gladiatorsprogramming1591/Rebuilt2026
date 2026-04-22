package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double RPS_RL;
    public double RPS_RF;
    public double RPS_LL;
    public double RPS_LF;
    public double RPM_RL;
    public double RPM_RF;
    public double RPM_LL;
    public double RPM_LF;
    public double appliedVolts_RL;
    public double appliedVolts_RF;
    public double appliedVolts_LL;
    public double appliedVolts_LF;
    public double motorTemp_RL;
    public double motorTemp_RF;
    public double motorTemp_LL;
    public double motorTemp_LF;
    public double supplyCurrent_RL;
    public double supplyCurrent_RF;
    public double supplyCurrent_LL;
    public double supplyCurrent_LF;
    public double torqueCurrentAmps_RL;
    public double torqueCurrentAmps_RF;
    public double torqueCurrentAmps_LL;
    public double torqueCurrentAmps_LF;
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
