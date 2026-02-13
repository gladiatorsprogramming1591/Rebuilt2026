package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerAppliedVolts = 0.0;

    public double intakeAppliedVolts = 0.0;
    public double deployAppliedVolts = 0.0;
  }

  /**
   * Refreshes the {@link IntakeIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(IntakeIOInputs inputs) {
    return false;
  }
  /**
   * sets the deployment motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setIntakeMotorVoltage(double volts) {}
  /**
   * sets the roller motor's voltage
   *
   * @param volts voltage value from -12 to 12
   */
  public default void setDeployMotorVoltage(double volts) {}
}
