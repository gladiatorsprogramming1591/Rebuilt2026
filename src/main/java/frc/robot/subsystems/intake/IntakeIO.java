package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the intake subsystem.
 *
 * <p>The intake subsystem owns behavior and writes desired outputs into {@link IntakeIOOutputs}.
 * Each IO implementation reads hardware sensors into {@link IntakeIOInputs} and applies the
 * requested outputs to real or simulated hardware.
 */
public interface IntakeIO {
  /** Sensor values read from intake hardware each loop. */
  @AutoLog
  public static class IntakeIOInputs {
    /** Slapdown deploy motor velocity in current mechanism units per second. */
    public double slapdownVelocity = 0.0;

    /** Slapdown deploy motor torque current in amps. */
    public double slapdownTorqueCurrent = 0.0;

    /** Slapdown deploy motor supply current in amps. */
    public double slapdownSupplyCurrent = 0.0;

    /** Slapdown deploy motor stator current in amps. */
    public double slapdownStatorCurrent = 0.0;

    /** True when the bottom/deployed slapdown limit sensor is active. */
    public boolean slapdownDown = false;

    /** True when the top/stowed slapdown limit sensor is active. */
    public boolean slapdownUp = false;

    /** Slapdown position after applying the current encoder offset. */
    public double slapdownPosition = 0.0;

    /** Raw slapdown motor encoder position before applying the offset. */
    public double slapdownRawPosition = 0.0;

    /** Offset applied to the raw slapdown encoder position. */
    public double slapdownEncoderOffset = 0.0;

    /** Left roller duty cycle output. */
    public double rollerLeftDutyCycle = 0.0;

    /** Right roller duty cycle output. */
    public double rollerRightDutyCycle = 0.0;

    /** Left roller RPS output. */
    public double rollerLeftRPS = 0.0;

    /** Right roller RPS output. */
    public double rollerRightRPS = 0.0;

    /** Left roller motor temperature in Celsius. */
    public double rollerLeftTemperature = 0.0;

    /** Right roller motor temperature in Celsius. */
    public double rollerRightTemperature = 0.0;

    /** Left roller supply current in amps. */
    public double rollerLeftSupplyCurrent = 0.0;

    /** Right roller supply current in amps. */
    public double rollerRightSupplyCurrent = 0.0;

    /** Left roller stator current in amps. */
    public double rollerLeftStatorCurrent = 0.0;

    /** Right roller stator current in amps. */
    public double rollerRightStatorCurrent = 0.0;

    /** Left roller torque current in amps. */
    public double rollerLeftTorqueCurrent = 0.0;

    /** Right roller torque current in amps. */
    public double rollerRightTorqueCurrent = 0.0;
  }

  /** Desired outputs written by the intake subsystem and applied by the IO implementation. */
  @AutoLog
  public static class IntakeIOOutputs {
    /** Desired open-loop roller output. */
    public double appliedRollerSpeed = 0.0;

    /** Desired open-loop slapdown output when slapdown mode is SPEED. */
    public double appliedSlapdownSpeed = 0.0;

    /** Desired slapdown position in current legacy slapdown units. */
    public double desiredSlapdownPosition = 0.0;

    /**
     * Desired slapdown stator current limit.
     *
     * <p>The subsystem can lower this during gentle movements, such as slow stowing while shooting.
     */
    public double slapdownStatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;

    public double deployKP = 0.0;
    public double deployKI = 0.0;
    public double deployKD = 0.0;
    public double deployKG = 0.0;
    public double deployFF = 0.0;

    public double stowKP = 0.0;
    public double stowKI = 0.0;
    public double stowKD = 0.0;
    public double stowKG = 0.0;
    public double stowFF = 0.0;
    public double stowMMAcceleration = 0.0;
    public double stowMMJerk = 0.0;

    public double stowFullKP = 0.0;
    public double stowFullKI = 0.0;
    public double stowFullKD = 0.0;
    public double stowFullKG = 0.0;
    public double stowFullFF = 0.0;
  }

  /**
   * Updates the latest intake sensor inputs.
   *
   * @param inputs container updated with the latest intake hardware state
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Applies the latest desired intake outputs.
   *
   * @param outputs latest requested intake outputs from the subsystem
   */
  public default void applyOutputs(IntakeIOOutputs outputs) {}

  /**
   * Optional connection check for hardware implementations.
   *
   * @param inputs latest intake inputs
   * @return true if the intake hardware is connected
   */
  public default boolean getIsConnected(IntakeIOInputs inputs) {
    return false;
  }
}