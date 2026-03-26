package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double deploySpeed = 0.0;
    public double deployTorqueCurrentFOC = 0.0;
    public double deploySupplyCurrent = 0.0;
    public double intakeLeftSpeed = 0.0;
    public double intakeRightSpeed = 0.0;
    public double deployPosition = 0.0;
    public double intakeLeftTemp = 0.0;
    public double intakeRightTemp = 0.0;
  }

  @AutoLog
  public static class IntakeIOOutputs {
    public double appliedIntakeSpeed = 0.0;
    public double appliedDeploySpeed = 0.0;
    public double appliedDeployCurrent = 0.0;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kFF = 0.0;
    public double desiredPosition = 0.0; 
  }

  /**
   * Refreshes the {@link IntakeIOInputs} object with the latest sensor readings and derived values.
   *
   * @param inputs container to populate
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}

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

  public default void runPosition(double positionRads) {}
}
