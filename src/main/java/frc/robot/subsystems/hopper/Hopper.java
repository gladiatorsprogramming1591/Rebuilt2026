package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_TABLE_KEY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the hopper belt.
 *
 * <p>The hopper is a single belt-floor mechanism. It can run forward to feed fuel into the shooter,
 * reverse to clear fuel, or run forward at a lower current limit while intaking to gently agitate
 * fuel into the hopper.
 */
public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private final HopperIOOutputsAutoLogged outputs = new HopperIOOutputsAutoLogged();

  private final Timer hopperEmptyTimer = new Timer();
  private boolean isHopperEmptyOverTime = false;

  /**
   * Creates a hopper subsystem using the provided hardware implementation.
   *
   * @param io hopper hardware abstraction
   */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  /**
   * Updates hopper inputs, updates the debounced empty state, logs subsystem state, and applies the
   * requested outputs to the IO layer.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    updateHopperEmptyOverTime();

    Logger.recordOutput(HOPPER_TABLE_KEY + "BeltSpeed", outputs.beltSpeed);
    Logger.recordOutput(
        HOPPER_TABLE_KEY + "UseBeltWhileIntakeCurrent", outputs.useBeltWhileIntakeCurrent);
    Logger.recordOutput(HOPPER_TABLE_KEY + "IsHopperEmptyOverTime", isHopperEmptyOverTime);

    io.applyOutputs(outputs);
  }

  /**
   * Runs the hopper belt forward at the normal current limit.
   *
   * <p>This is the main command used while shooting.
   *
   * @return command that runs the hopper belt forward until interrupted
   */
  public Command startBeltMotors() {
    return runEnd(this::runBeltForward, this::stopBelt);
  }

  /**
   * Runs the hopper belt forward using the lower intaking/agitation current limit.
   *
   * <p>This can help move fuel into the hopper while intaking without using the full belt current
   * limit.
   *
   * @return command that runs the hopper belt in intake/agitation mode until interrupted
   */
  public Command runBeltWhileIntaking() {
    return runEnd(this::runBeltForwardWithIntakeCurrent, this::stopBelt);
  }

  /**
   * Runs the hopper belt in reverse at the normal current limit.
   *
   * <p>This is intended for clearing or backing fuel out of the hopper.
   *
   * @return command that reverses the hopper belt until interrupted
   */
  public Command reverseBeltMotors() {
    return runEnd(this::runBeltReverse, this::stopBelt);
  }

  /**
   * Keeps the hopper belt stopped.
   *
   * <p>This is intended to be the hopper default command.
   *
   * @return command that continuously stops the hopper belt
   */
  public Command stopBeltMotors() {
    return run(this::stopBelt);
  }

  /**
   * Returns the debounced hopper empty state.
   *
   * <p>The raw CANrange empty state must be true for {@link HopperConstants#MIN_EMPTY_DURATION}
   * before this returns true. If the CANrange is disconnected, this returns false.
   *
   * @return true when the hopper has been empty for the configured minimum duration
   */
  public boolean isHopperEmpty() {
    return isHopperEmptyOverTime;
  }

  /**
   * Updates the debounced hopper empty state.
   *
   * <p>The hopper is only considered empty when the empty sensor is connected and reports empty for
   * the configured minimum duration.
   */
  private void updateHopperEmptyOverTime() {
    if (inputs.hopperEmptySensorConnected && inputs.hopperEmpty) {
      if (!hopperEmptyTimer.isRunning()) {
        hopperEmptyTimer.start();
      }
    } else {
      hopperEmptyTimer.stop();
      hopperEmptyTimer.reset();
    }

    isHopperEmptyOverTime = hopperEmptyTimer.hasElapsed(HopperConstants.MIN_EMPTY_DURATION);
  }

  /** Runs the belt forward using the normal current limit. */
  private void runBeltForward() {
    outputs.useBeltWhileIntakeCurrent = false;
    outputs.beltSpeed = -HopperConstants.getBeltMotorSpeed();
  }

  /** Runs the belt forward using the lower intaking/agitation current limit. */
  private void runBeltForwardWithIntakeCurrent() {
    outputs.useBeltWhileIntakeCurrent = true;
    outputs.beltSpeed = -HopperConstants.getBeltMotorSpeed();
  }

  /** Runs the belt in reverse using the normal current limit. */
  private void runBeltReverse() {
    outputs.useBeltWhileIntakeCurrent = false;
    outputs.beltSpeed = HopperConstants.getBeltMotorSpeed();
  }

  /** Stops the belt and restores the normal current limit mode. */
  private void stopBelt() {
    outputs.useBeltWhileIntakeCurrent = false;
    outputs.beltSpeed = 0.0;
  }
}
