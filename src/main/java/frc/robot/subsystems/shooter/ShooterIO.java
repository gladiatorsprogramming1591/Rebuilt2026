package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftLeaderMotorTemp;
    public double rightLeaderMotorTemp;
    public double leftFollowerMotorTemp;
    public double rightFollowerMotorTemp;
    public double velocityRPM;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double appliedVoltage;
  }

  public class ShooterIOOutputs {
    public double desiredVelocityRPM = 0.0; // so far only used for sim
    public double kP;
    public double kI;
    public double kD;
    public double kV;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}

  /**
   * setting the shooter motor voltage
   *
   * @param shooterVelocity voltage to set from -12 to 12
   */
  public default void runShooter(double shooterVelocity) {}

  public default void runShooterVelocity(double shooterVelocity) {}

  public default boolean shooterAtVelocity(double shooterVelocity) {
    return false;
  }

  public default void setShooterMotorRPM(double rpm) {}
}
