package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;

/**
 * Simple simulated hopper IO implementation.
 *
 * <p>This is not intended to be a full physics simulation. It exists so the hopper subsystem can
 * run in sim/replay without real hardware while following the same input/output structure as the
 * real IO implementation.
 */
public class HopperIOSim implements HopperIO {
  private double beltSpeed = 0.0;
  private boolean usingIntakeCurrentLimit = false;

  /**
   * Creates a simple simulated hopper with the belt stopped and an empty-sensor reading present.
   */
  public HopperIOSim() {}

  /**
   * Updates the simulated hopper sensor values.
   *
   * @param inputs container updated with the latest simulated hopper state
   */
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.beltCurrent = Math.abs(beltSpeed) * HopperConstants.BELT_CURRENT_LIMIT;
    inputs.beltVelocity = beltSpeed;
    inputs.hopperEmptySensorConnected = true;
    inputs.hopperEmptyDistance = 1.0;
    inputs.hopperEmpty = true;
  }

  /**
   * Applies the requested hopper output to the simple simulation state.
   *
   * @param outputs latest requested hopper outputs from the subsystem
   */
  @Override
  public void applyOutputs(HopperIOOutputs outputs) {
    usingIntakeCurrentLimit = outputs.useBeltWhileIntakeCurrent;
    beltSpeed = MathUtil.clamp(outputs.beltSpeed, -1.0, 1.0);
  }
}
