package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import java.util.function.BooleanSupplier;

/**
 * Simple simulated hood IO implementation.
 *
 * <p>This is not intended to be a full physics simulation. It exists so the hood subsystem can run in
 * sim/replay without real hardware while still following the same input/output structure as the real
 * IO implementation.
 */
public class HoodIOSim implements HoodIO {
  private double speed = 0.0;
  private double angle = 0.0;

  /** Creates a simple simulated hood starting at zero. */
  public HoodIOSim() {}

  /**
   * Updates the simulated hood sensor values.
   *
   * @param inputs container updated with the latest simulated hood state
   */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodSpeed = speed;
    inputs.hoodAngle = angle;

    inputs.hoodLimitRaw = !isBottomLimitTripped();
    inputs.hoodLimitTripped = isBottomLimitTripped();
    inputs.hoodLimitSet = inputs.hoodLimitTripped;
  }

  /**
   * Applies the requested hood output to the simple simulation state.
   *
   * <p>Position mode directly moves the simulated hood to the requested angle. Speed mode integrates
   * the requested open-loop output over one normal robot loop.
   *
   * @param outputs latest requested hood outputs from the subsystem
   */
  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodMode.POSITION) {
      angle =
          MathUtil.clamp(
              outputs.desiredHoodAngle,
              HoodConstants.HOOD_LOWER_LIMIT,
              HoodConstants.HOOD_UPPER_LIMIT);
      speed = 0.0;
      return;
    }

    speed =
        MathUtil.clamp(
            outputs.desiredHoodSpeed,
            -HoodConstants.HOOD_MAX_SPEED,
            HoodConstants.HOOD_MAX_SPEED);

    angle += speed * 0.02 / HoodConstants.HOOD_MOTOR_REDUCTION;
    angle = MathUtil.clamp(angle, HoodConstants.HOOD_LOWER_LIMIT, HoodConstants.HOOD_UPPER_LIMIT);
  }

  /**
   * Seeds the simulated hood encoder to zero.
   *
   * <p>This mirrors the real IO behavior where zeroing resets the encoder without physically moving
   * the mechanism.
   */
  @Override
  public void zeroHood() {
    angle = 0.0;
    speed = 0.0;
  }

  /**
   * Returns whether the simulated hood encoder is near zero.
   *
   * @return supplier that is true when the simulated hood is within the configured zero tolerance
   */
  @Override
  public BooleanSupplier isHoodWithinZeroTolerance() {
    return () -> Math.abs(angle) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  /**
   * Returns whether the simulated hood is at the bottom hard stop.
   *
   * @return true when the simulated hood is at or below the lower limit
   */
  @Override
  public boolean isHoodAtTrueZero() {
    return isBottomLimitTripped();
  }

  /**
   * Resets simulated zeroing fallback state.
   *
   * <p>The simple sim has no timer-backed fallback, so this is intentionally empty.
   */
  @Override
  public void resetHoodTimer() {}

  private boolean isBottomLimitTripped() {
    return angle <= HoodConstants.HOOD_LOWER_LIMIT;
  }
}