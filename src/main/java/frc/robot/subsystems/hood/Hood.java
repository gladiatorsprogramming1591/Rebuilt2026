package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_TABLE_KEY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodIO.HoodMode;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the adjustable shooter hood.
 *
 * <p>The hood uses a motor-relative encoder, so its position is only trusted after the mechanism
 * has found the bottom hard stop. While the hood is not zeroed, position control is blocked and the
 * subsystem only allows slow open-loop zeroing. Once zeroed, the hood can safely use closed-loop
 * position control for shooting and for returning to zero.
 */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputsAutoLogged outputs = new HoodIOOutputsAutoLogged();

  private boolean hasInitiallyBeenZeroed = false;
  private boolean hasReducedCurrentLimit = false;
  private boolean hasAppliedZeroAtCurrentHardStop = false;

  private static final LoggedTunableNumber goalPosition =
      new LoggedTunableNumber("Hood/GoalPosition", 100.0, Constants.Tuning.HOOD);

  private static final LoggedTunableNumber upKP =
      new LoggedTunableNumber("Hood/Up/kP", HoodConstants.HOOD_UP_KP, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKI =
      new LoggedTunableNumber("Hood/Up/kI", HoodConstants.HOOD_UP_KI, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKD =
      new LoggedTunableNumber("Hood/Up/kD", HoodConstants.HOOD_UP_KD, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKS =
      new LoggedTunableNumber("Hood/Up/kS", HoodConstants.HOOD_UP_KS, Constants.Tuning.HOOD);

  private static final LoggedTunableNumber downKP =
      new LoggedTunableNumber("Hood/Down/kP", HoodConstants.HOOD_DOWN_KP, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKI =
      new LoggedTunableNumber("Hood/Down/kI", HoodConstants.HOOD_DOWN_KI, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKD =
      new LoggedTunableNumber("Hood/Down/kD", HoodConstants.HOOD_DOWN_KD, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKS =
      new LoggedTunableNumber("Hood/Down/kS", HoodConstants.HOOD_DOWN_KS, Constants.Tuning.HOOD);

  /**
   * Creates a hood subsystem using the provided hardware implementation.
   *
   * @param io hood hardware abstraction
   */
  public Hood(HoodIO io) {
    this.io = io;
  }

  /**
   * Updates hood inputs, logs subsystem state, copies tunable values into outputs, and applies the
   * requested outputs to the IO layer.
   *
   * <p>This method intentionally does not decide whether the hood is zeroed. Zero is only accepted by
   * {@link #runHoodToZero()} or {@link #zeroHood()} so that the subsystem does not accidentally trust
   * the encoder just because the hood appears stopped.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    outputs.hasBeenZeroed = hasInitiallyBeenZeroed;

    outputs.upKP = upKP.get();
    outputs.upKI = upKI.get();
    outputs.upKD = upKD.get();
    outputs.upKS = upKS.get();

    outputs.downKP = downKP.get();
    outputs.downKI = downKI.get();
    outputs.downKD = downKD.get();
    outputs.downKS = downKS.get();

    Logger.recordOutput(HOOD_TABLE_KEY + "DesiredAngle", outputs.desiredHoodAngle);
    Logger.recordOutput(HOOD_TABLE_KEY + "DesiredSpeed", outputs.desiredHoodSpeed);
    Logger.recordOutput(HOOD_TABLE_KEY + "Mode", outputs.mode.toString());
    Logger.recordOutput(HOOD_TABLE_KEY + "HasInitiallyBeenZeroed", hasInitiallyBeenZeroed);
    Logger.recordOutput(HOOD_TABLE_KEY + "HasReducedCurrentLimit", hasReducedCurrentLimit);
    Logger.recordOutput(
        HOOD_TABLE_KEY + "HasAppliedZeroAtCurrentHardStop", hasAppliedZeroAtCurrentHardStop);

    io.applyOutputs(outputs);
  }

  /**
   * Returns whether the hood encoder has been zeroed since robot boot.
   *
   * @return true after the hood has reached mechanical zero and the encoder has been reset
   */
  public BooleanSupplier getHasInitiallyBeenZeroed() {
    return () -> hasInitiallyBeenZeroed;
  }

  /**
   * Returns the hood to its mechanical zero position.
   *
   * <p>This command is intended to be the hood default command. It has three phases:
   *
   * <ol>
   *   <li>If the hood is at the hard stop, apply encoder zero once.
   *   <li>If the hood has already been zeroed and is far from zero, move to an approach angle using
   *       position control.
   *   <li>Once near zero, or before the encoder is trusted, creep downward open-loop until the zero
   *       sensor or fallback stall detection says the hood is at true zero.
   * </ol>
   *
   * <p>The approach angle prevents the hood from slamming into the hard stop from far away while
   * still allowing a gentle final zeroing move.
   *
   * @return command that returns the hood to zero while scheduled
   */
  public Command runHoodToZero() {
    return run(this::runHoodToZeroLoop)
        .finallyDo(
            () -> {
              stopHood();
              setDefaultStatorCurrentLimit();
              io.resetHoodTimer();
            });
  }

  /**
   * Runs the hood to a requested position.
   *
   * <p>The request is ignored until the hood has been zeroed. This prevents closed-loop position
   * control from using an untrusted encoder position after boot.
   *
   * @param angleSupplier target hood angle in the current legacy hood units
   * @return command that holds the requested hood position while scheduled
   */
  public Command runHoodPosition(DoubleSupplier angleSupplier) {
    return run(() -> setHoodPositionIfZeroed(angleSupplier.getAsDouble()));
  }

  /**
   * Runs the hood to the current shooter-calculated target angle.
   *
   * <p>When hood tuning is enabled, this command uses the tunable dashboard goal instead of the
   * shooter map. Like all position commands, it will not move the hood until the hood has been
   * zeroed.
   *
   * @return command that continuously updates the hood target while scheduled
   */
  public Command runHoodTarget() {
    return run(
        () -> {
          double desiredAngle =
              Constants.Tuning.HOOD
                  ? goalPosition.getAsDouble()
                  : ShooterCalculation.getInstance().getParameters().hoodAngle();

          setHoodPositionIfZeroed(desiredAngle);
        });
  }

  /**
   * Returns whether the hood is at its current commanded position.
   *
   * <p>This returns false until the hood has been zeroed because the encoder position is not trusted
   * before then.
   *
   * @return true when the hood is zeroed and within the configured angle tolerance
   */
  public BooleanSupplier isHoodAtAngle() {
    return () -> {
      boolean atAngle =
          hasInitiallyBeenZeroed
              && Math.abs(outputs.desiredHoodAngle - inputs.hoodAngle)
                  < HoodConstants.HOOD_ANGLE_TOLERANCE;

      Logger.recordOutput(HOOD_TABLE_KEY + "AtAngle", atAngle);
      return atAngle;
    };
  }

  /**
   * Runs one loop of the hood zeroing command.
   *
   * <p>The order matters:
   *
   * <ol>
   *   <li>If the hood is already at the hard stop, apply encoder zero.
   *   <li>If the encoder is trusted and the hood is still far from zero, move to the approach angle
   *       with position control.
   *   <li>Otherwise, slowly creep downward until the hard stop is detected.
   * </ol>
   */
  private void runHoodToZeroLoop() {
    // First priority: if the hood is physically at zero, stop and trust that position.
    if (io.isHoodAtTrueZero()) {
      applyZeroAtHardStop();
      return;
    }

    // The hood has moved away from the hard stop, so allow zero to be applied again next time.
    hasAppliedZeroAtCurrentHardStop = false;

    // If the encoder is already trusted, use position control for the long move down.
    if (shouldMoveToZeroApproachAngle()) {
      moveToZeroApproachAngle();
      return;
    }

    // If the encoder is not trusted, or we are already near zero, creep into the hard stop.
    creepTowardZero();
  }

  /**
   * Returns whether the hood should use position control to move near zero before creeping into the
   * hard stop.
   *
   * <p>This is only allowed after the hood has already been zeroed once. Before that, the encoder
   * position is not trusted, so the hood must use slow open-loop zeroing instead.
   *
   * @return true when the hood is zeroed and far enough above the approach angle
   */
  private boolean shouldMoveToZeroApproachAngle() {
    return hasInitiallyBeenZeroed
        && inputs.hoodAngle
            > HoodConstants.HOOD_ZEROING_APPROACH_ANGLE
                + HoodConstants.HOOD_ZEROING_APPROACH_TOLERANCE;
  }

  /**
   * Moves the hood to the zeroing approach angle using closed-loop position control.
   *
   * <p>This is used only after the hood has already been zeroed. It lets the hood return most of the
   * way to zero without driving into the hard stop from far away.
   */
  private void moveToZeroApproachAngle() {
    setDefaultStatorCurrentLimit();

    outputs.mode = HoodMode.POSITION;
    outputs.desiredHoodAngle = HoodConstants.HOOD_ZEROING_APPROACH_ANGLE;
    outputs.desiredHoodSpeed = 0.0;
  }

  /**
   * Slowly drives the hood downward toward the zero hard stop.
   *
   * <p>This is the only zeroing motion used before the encoder is trusted. It is also used for the
   * final approach after closed-loop control has moved the hood near zero.
   */
  private void creepTowardZero() {
    setReducedStatorCurrentLimit();

    outputs.mode = HoodMode.SPEED;
    outputs.desiredHoodAngle = HoodConstants.HOOD_ZEROING_APPROACH_ANGLE;
    outputs.desiredHoodSpeed = HoodConstants.HOOD_ZEROING_SPEED;
  }

  /**
   * Accepts the current hood position as mechanical zero.
   *
   * <p>This should only run when the bottom limit sensor or fallback stall detection says the hood is
   * physically at the zero hard stop. The encoder is only reset once while the hood remains on the
   * same hard-stop contact.
   */
  private void applyZeroAtHardStop() {
    stopHood();
    setDefaultStatorCurrentLimit();
    io.resetHoodTimer();

    if (!hasAppliedZeroAtCurrentHardStop) {
      io.zeroHood();
      hasAppliedZeroAtCurrentHardStop = true;
    }

    hasInitiallyBeenZeroed = true;
    outputs.hasBeenZeroed = true;
  }

  /**
   * Commands the hood to a position only if the encoder has been zeroed.
   *
   * <p>If the hood has not been zeroed, this stops the hood instead of allowing closed-loop control
   * to use an untrusted encoder position.
   *
   * @param angle requested hood angle in current legacy hood units
   */
  private void setHoodPositionIfZeroed(double angle) {
    if (!hasInitiallyBeenZeroed) {
      stopHood();
      return;
    }

    outputs.mode = HoodMode.POSITION;
    outputs.desiredHoodAngle =
        MathUtil.clamp(angle, HoodConstants.HOOD_LOWER_LIMIT, HoodConstants.HOOD_UPPER_LIMIT);
    outputs.desiredHoodSpeed = 0.0;
  }

  /**
   * Runs the hood in open-loop speed mode.
   *
   * <p>The requested speed is clamped before being passed to the IO layer.
   *
   * @param speed requested percent output
   */
  private void runHoodOpenLoop(double speed) {
    outputs.mode = HoodMode.SPEED;
    outputs.desiredHoodSpeed =
        MathUtil.clamp(speed, -HoodConstants.HOOD_MAX_SPEED, HoodConstants.HOOD_MAX_SPEED);
  }

  /** Stops the hood by commanding zero open-loop output. */
  private void stopHood() {
    outputs.mode = HoodMode.SPEED;
    outputs.desiredHoodSpeed = 0.0;
  }

  /**
   * Applies the reduced stator current limit used during slow zeroing.
   *
   * <p>The current limit is only sent when the state changes so the subsystem does not repeatedly
   * reconfigure the motor controller every loop.
   */
  private void setReducedStatorCurrentLimit() {
    if (!hasReducedCurrentLimit) {
      hasReducedCurrentLimit = true;
      io.setStatorCurrentLimit(HoodConstants.HOOD_ZEROING_STATOR_CURRENT_LIMIT);
    }
  }

  /**
   * Restores the normal hood stator current limit.
   *
   * <p>The current limit is only sent when the state changes so the subsystem does not repeatedly
   * reconfigure the motor controller every loop.
   */
  private void setDefaultStatorCurrentLimit() {
    if (hasReducedCurrentLimit) {
      hasReducedCurrentLimit = false;
      io.setStatorCurrentLimit(HoodConstants.HOOD_STATOR_CURRENT_LIMIT);
    }
  }

  /**
   * Manually runs the hood upward while the command is held.
   *
   * <p>This is intended for driver/operator control and debugging. It does not require the hood to
   * be zeroed because it is open-loop control.
   *
   * @return command that runs the hood upward until interrupted
   */
  public Command runHoodUp() {
    return runEnd(
        () -> runHoodOpenLoop(HoodConstants.HOOD_UP_SPEED),
        this::stopHood);
  }

  /**
   * Manually runs the hood downward while the command is held.
   *
   * <p>This is intended for driver/operator control and debugging. It does not require the hood to
   * be zeroed because it is open-loop control.
   *
   * @return command that runs the hood downward until interrupted
   */
  public Command runHoodDown() {
    return runEnd(
        () -> runHoodOpenLoop(HoodConstants.HOOD_DOWN_SPEED),
        this::stopHood);
  }

  /**
   * Manually resets the hood encoder to zero at the current position.
   *
   * <p>This is an operator/debug command. It does not physically move the hood to the hard stop, so
   * it should only be used when the operator knows the hood is already at the correct zero position.
   *
   * @return instant command that seeds the hood encoder to zero
   */
  public Command zeroHood() {
    return runOnce(
        () -> {
          io.zeroHood();
          hasInitiallyBeenZeroed = true;
          hasAppliedZeroAtCurrentHardStop = true;
          outputs.hasBeenZeroed = true;
        });
  }
}