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
    public boolean isDeployDown = false;
    public boolean isDeployUp = false;
    public double encoderOffset = 0.0;
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
   * Applies the latest tunable TalonFX configurations to the <b>deploy motor</b>.
   * <p>
   * Only applies the configuration when the Smartdashboard boolean {@value #updateDeployConfigName} is changed from false to true (i.e. rising edge).
   * 
   * @param outputs Intake outputs where the tunable configurations are accessable
   * @see {@link #createTunedDeployMotorConfig(frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs) createTunedDeployMotorConfig()}
   * @see frc.robot.util.LoggedTunableNumber LoggedTunableNumber
   */
  public default void tuneDeployMotorConfigs(IntakeIOOutputs outputs) {}

  /**
   * Convenience connection check if an implementation prefers reading from {@code inputs}.
   *
   * @param inputs latest inputs
   * @return true if connected
   */
  public default boolean getIsConnected(IntakeIOInputs inputs) {
    return false;
  }
}
