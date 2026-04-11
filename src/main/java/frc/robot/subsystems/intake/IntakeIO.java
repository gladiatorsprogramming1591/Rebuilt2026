package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double slapdownSpeed = 0.0;
    public double slapdownTorqueCurrentFOC = 0.0;
    public double slapdownSupplyCurrent = 0.0;
    public double rollerLeftSpeed = 0.0;
    public double rollerRightSpeed = 0.0;
    public double rollerLeftTemp = 0.0;
    public double rollerRightTemp = 0.0;
    public boolean isSlapdownDown = false;
    public boolean isSlapdownUp = false;
    public double position = 0.0;
    public double encoderOffset = 0.0;
  }

  @AutoLog
  public static class IntakeIOOutputs {
    public double appliedRollerSpeed = 0.0;
    public double appliedSlapdownSpeed = 0.0;
    public double appliedSlapdownCurrent = 0.0;
    public double kdeployP = 0.0;
    public double kdeployI = 0.0;
    public double kdeployD = 0.0;
    public double kdeployFF = 0.0;
    public double kstowP = 0.0;
    public double kstowI = 0.0;
    public double kstowD = 0.0;
    public double kstowFF = 0.0;
    public double kstowMMAcceleration = 0.0;
    public double kstowMMJerk = 0.0;
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
