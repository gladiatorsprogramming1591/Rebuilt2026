package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotState;

/**
 * Simple simulated intake IO implementation.
 *
 * <p>This is not intended to be a full physics simulation. It exists so the intake subsystem can run
 * in sim/replay without real hardware while following the same input/output structure as the real IO
 * implementation.
 */
public class IntakeIOSim implements IntakeIO {
  private double rollerSpeed = 0.0;
  private double slapdownSpeed = 0.0;
  private double slapdownPosition = IntakeConstants.UP;
  private double slapdownEncoderOffset = 0.0;
  private double slapdownStatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;

  /** Creates a simple simulated intake starting stowed. */
  public IntakeIOSim() {}

  /**
   * Updates the simulated intake sensor values.
   *
   * @param inputs container updated with the latest simulated intake state
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.slapdownVelocity = slapdownSpeed;
    inputs.slapdownSupplyCurrent =
        isAtSlapdownLimit() ? IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD : 1.0;
    inputs.slapdownStatorCurrent =
        Math.min(Math.abs(slapdownSpeed) * slapdownStatorCurrentLimit, slapdownStatorCurrentLimit);
    inputs.slapdownTorqueCurrent = inputs.slapdownStatorCurrent;

    inputs.slapdownDown = slapdownPosition == IntakeConstants.DOWN;
    inputs.slapdownUp = slapdownPosition == IntakeConstants.UP;
    inputs.slapdownPosition = slapdownPosition;
    inputs.slapdownRawPosition = slapdownPosition - slapdownEncoderOffset;
    inputs.slapdownEncoderOffset = slapdownEncoderOffset;

    inputs.rollerLeftDutyCycle = rollerSpeed;
    inputs.rollerRightDutyCycle = rollerSpeed;
    inputs.rollerLeftSupplyCurrent = Math.abs(rollerSpeed) * 10.0;
    inputs.rollerRightSupplyCurrent = Math.abs(rollerSpeed) * 10.0;
    inputs.rollerLeftStatorCurrent = inputs.rollerLeftSupplyCurrent;
    inputs.rollerRightStatorCurrent = inputs.rollerRightSupplyCurrent;
    inputs.rollerLeftTorqueCurrent = inputs.rollerLeftSupplyCurrent;
    inputs.rollerRightTorqueCurrent = inputs.rollerRightSupplyCurrent;
    inputs.rollerLeftTemperature = 25.0;
    inputs.rollerRightTemperature = 25.0;
  }

  /**
   * Applies the requested intake output to the simple simulation state.
   *
   * @param outputs latest requested intake outputs from the subsystem
   */
  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    rollerSpeed = MathUtil.clamp(outputs.appliedRollerSpeed, -1.0, 1.0);
    slapdownStatorCurrentLimit = outputs.slapdownStatorCurrentLimit;

    switch (RobotState.getSlapdownMode()) {
      case DEPLOY_POSITION:
      case STOW_POSITION:
      case BUMP_POSITION:
        slapdownPosition =
            MathUtil.clamp(
                outputs.desiredSlapdownPosition,
                IntakeConstants.MIN_ANGLE,
                IntakeConstants.MAX_ANGLE);
        slapdownSpeed = 0.0;
        break;

      case SPEED:
        slapdownSpeed = outputs.appliedSlapdownSpeed;
        slapdownPosition += slapdownSpeed * 0.02;
        slapdownPosition =
            MathUtil.clamp(
                slapdownPosition, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
        break;

      case OFF:
        slapdownSpeed = 0.0;
        break;

      default:
        slapdownSpeed = 0.0;
        break;
    }

    updateEncoderOffsetAtLimits();
  }

  /** Updates the simulated encoder offset when the slapdown reaches either hard stop. */
  private void updateEncoderOffsetAtLimits() {
    if (slapdownPosition == IntakeConstants.DOWN) {
      slapdownEncoderOffset = -slapdownPosition;
    } else if (slapdownPosition == IntakeConstants.UP) {
      slapdownEncoderOffset = -(slapdownPosition - IntakeConstants.UP);
    }
  }

  /**
   * Returns whether the simulated slapdown is at either hard stop.
   *
   * @return true when the slapdown is fully deployed or fully stowed
   */
  private boolean isAtSlapdownLimit() {
    return slapdownPosition == IntakeConstants.DOWN || slapdownPosition == IntakeConstants.UP;
  }
}