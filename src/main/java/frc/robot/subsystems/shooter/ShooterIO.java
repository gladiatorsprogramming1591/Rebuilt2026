package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
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
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;
    public double kMMAcceleration;
    public double kMMJerk;
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

  public default BooleanSupplier rightShooterAtVelocity() {
    return () -> false;
  }

  public default BooleanSupplier leftShooterAtVelocity() {
    return () -> false;
  }

  public default BooleanSupplier bothShootersAtVelocity() {
    return () -> false;
  }

  public default void setShooterMotorRPM(double rpm) {}
}
