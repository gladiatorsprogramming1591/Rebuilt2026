package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;

/**
 * Simple simulated kicker IO implementation.
 *
 * <p>This is not intended to be a full physics simulation. It exists so the kicker subsystem can
 * run in sim/replay without real hardware while following the same input/output structure as the
 * real IO implementation.
 */
public class KickerIOSim implements KickerIO {
  private double speed = 0.0;

  /** Creates a simple simulated kicker with the motor stopped. */
  public KickerIOSim() {}

  /**
   * Updates the simulated kicker sensor values.
   *
   * @param inputs container updated with the latest simulated kicker state
   */
  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.primaryConnected = true;
    inputs.primaryDutyCycle = speed;
    inputs.primaryVelocity = speed;
    inputs.primaryAppliedVolts = speed * 12.0;
    inputs.primarySupplyCurrent = Math.abs(speed) * KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT;
    inputs.primaryStatorCurrent = inputs.primarySupplyCurrent;
    inputs.primaryTorqueCurrent = inputs.primarySupplyCurrent;
    inputs.primaryTemperature = 25.0;

    inputs.secondaryConnected = KickerConstants.HAS_SECOND_KICKER_MOTOR;
    inputs.secondaryDutyCycle = KickerConstants.HAS_SECOND_KICKER_MOTOR ? speed : 0.0;
    inputs.secondaryVelocity = KickerConstants.HAS_SECOND_KICKER_MOTOR ? speed : 0.0;
    inputs.secondaryAppliedVolts = KickerConstants.HAS_SECOND_KICKER_MOTOR ? speed * 12.0 : 0.0;
    inputs.secondarySupplyCurrent =
        KickerConstants.HAS_SECOND_KICKER_MOTOR
            ? Math.abs(speed) * KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT
            : 0.0;
    inputs.secondaryStatorCurrent = inputs.secondarySupplyCurrent;
    inputs.secondaryTorqueCurrent = inputs.secondarySupplyCurrent;
    inputs.secondaryTemperature = KickerConstants.HAS_SECOND_KICKER_MOTOR ? 25.0 : 0.0;
  }

  /**
   * Applies the requested simulated kicker output.
   *
   * @param outputs latest requested kicker outputs from the subsystem
   */
  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    speed = MathUtil.clamp(outputs.desiredKickerSpeed, -1.0, 1.0);
  }
}
