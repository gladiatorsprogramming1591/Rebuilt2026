package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double deploySpeed = 0.0;
    public double deployTorqueCurrentFOC = 0.0;
    public double intakeSpeed = 0.0;
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
   * sets the deployment motor's duty cycle
   *
   * @param speed duty cycle value from -1 to 1
   */
  public default void setDeploySpeed(double speed) {}
  /**
   * sets the deploy motor's torque current (FOC)
   *
   * @param current current value (in Amps)
   */
  public default void setDeployTorqueCurrentFOC(double current) {}
  /**
   * sets the intake motor's duty cycle
   *
   * @param speed duty cycle value from -1 to 1
   */
  public default void setIntakeSpeed(double speed) {}
  /** stops the deploy motor */
  public default void stopDeployMotor() {}
  /** stops the intake motor */
  public default void stopIntakeMotor() {}
}
