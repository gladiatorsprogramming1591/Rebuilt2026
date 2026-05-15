package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kdeployTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowFullTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowTableKey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.RollerModeState;
import frc.robot.RobotState.SlapdownModeState;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the intake rollers and slapdown arm.
 *
 * <p>Name conventions:
 *
 * <ul>
 *   <li><b>Intake:</b> whole subsystem
 *   <li><b>Slapdown:</b> pivoting intake arm
 *   <li><b>Deploy:</b> full extension toward the floor
 *   <li><b>Stow:</b> full retraction into the frame perimeter
 *   <li><b>Rollers:</b> rotating tubes that move fuel into the hopper
 * </ul>
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();

  private boolean stopSlapdownOnCurrentSpike = false;
  private boolean isSlapdownStopped = true;
  private boolean overrideRollerSpeed = false;

  private boolean rollerBoostActive = false;
  private boolean rollerWasRequested = false;
  private double rollerRequestStartTimestamp = 0.0;
  private double rollerHighCurrentStartTimestamp = Double.NaN;
  private double rollerBoostUntilTimestamp = 0.0;

  @AutoLogOutput private double manualAngle = 0.0;
  @AutoLogOutput private double requestedRollerSpeed = 0.0;

  /**
   * Creates an intake subsystem using the provided hardware implementation.
   *
   * @param io intake hardware abstraction
   */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  /**
   * Updates intake inputs, updates tunable outputs, applies requested outputs, and stops the
   * slapdown when a configured stop condition is reached.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    updateTunableOutputs();
    updateRollerOutput();
    logOutputs();

    io.applyOutputs(outputs);

    stopSlapdownIfNeeded();
  }

  /**
   * Returns the larger roller stator current.
   *
   * <p>Using max current is more useful than average current because one jammed roller should be
   * enough to request boost.
   *
   * @return max absolute roller stator current
   */
  private double getMaxRollerStatorCurrent() {
    return Math.max(
        Math.abs(inputs.rollerLeftStatorCurrent), Math.abs(inputs.rollerRightStatorCurrent));
  }

  /**
   * Deploys the slapdown to the down position.
   *
   * <p>The command finishes when {@link #deployStop()} marks the slapdown stopped. That happens when
   * the deploy limit sensor trips or the slapdown current reaches the stop threshold.
   *
   * @return command that deploys the slapdown
   */
  public Command deploy() {
    return runOnce(
            () ->
                requestSlapdownPosition(
                    IntakeConstants.DOWN, SlapdownModeState.DEPLOY_POSITION, true))
        .andThen(new WaitUntilCommand(() -> isSlapdownStopped));
  }

  /**
   * Stows the slapdown to the up position.
   *
   * <p>The command finishes when {@link #deployStop()} marks the slapdown stopped. That happens when
   * the stow limit sensor trips or the slapdown current reaches the stop threshold.
   *
   * @return command that stows the slapdown
   */
  public Command stow() {
    return runOnce(
            () -> requestSlapdownPosition(IntakeConstants.UP, SlapdownModeState.STOW_POSITION, true))
        .andThen(new WaitUntilCommand(() -> isSlapdownStopped));
  }

  /**
   * Deploys the slapdown and then runs the rollers once the deploy sensor reports down.
   *
   * @return command that deploys the intake and then runs the rollers
   */
  public Command deployAndRunRoller() {
    return deploy().andThen(new WaitUntilCommand(() -> inputs.slapdownDown)).andThen(runRoller());
  }

  /**
   * Moves the slapdown to the bump/intermediate position.
   *
   * @return command that holds the bump position while scheduled
   */
  public Command stowBump() {
    return runEnd(
        () -> requestSlapdownPosition(IntakeConstants.BUMP, SlapdownModeState.BUMP_POSITION, false),
        this::stopSlapdown);
  }

  /**
   * Moves the slapdown to the shooting stop position and stops the rollers.
   *
   * <p>This is the position-control shooting stow option. It commands the slapdown directly to
   * {@link IntakeConstants#SHOOTING_STOP}.
   *
   * @return command that holds the shooting stop position while scheduled
   */
  public Command stowWhileShooting() {
    return runEnd(
        () -> {
          requestSlapdownPosition(
              IntakeConstants.SHOOTING_STOP, SlapdownModeState.BUMP_POSITION, false);
          setRequestedRollerSpeed(0.8);
        },
        () -> {
          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

    /**
   * Pulses the slapdown while shooting, then falls back to the normal slow shooting curl.
   *
   * <p>The inward part of each pulse uses the existing shooting slow-stow speed. The short outward
   * relief pulse backs pressure off the hopper so fuel can settle instead of staying constantly
   * compressed. After the configured number of pulses, this behaves like the normal slow shooting
   * curl so the intake still ends in the normal shooting position.
   *
   * @return command that agitates the intake briefly while shooting
   */
  public Command agitateWhileShooting() {
    final Timer timer = new Timer();
    final int[] completedCurls = new int[1];
    final boolean[] curlingIn = new boolean[1];

    return runEnd(
        () -> {
          if (!timer.isRunning()) {
            completedCurls[0] = 0;
            curlingIn[0] = true;
            timer.restart();
          }

          setRequestedRollerSpeed(0.8);

          int requestedCurls =
              Math.max(0, (int) Math.round(IntakeConstants.shootingAgitateCurlCount.getAsDouble()));

          if (completedCurls[0] >= requestedCurls) {
            requestSlapdownSlowStowSpeed();
            return;
          }

          if (curlingIn[0]) {
            requestSlapdownSlowStowSpeed();

            if (timer.hasElapsed(IntakeConstants.shootingAgitateCurlSeconds.getAsDouble())
                || inputs.slapdownUp
                || inputs.slapdownPosition <= IntakeConstants.SHOOTING_STOP) {
              curlingIn[0] = false;
              timer.restart();
            }

            return;
          }

          requestShootingAgitateRelief();

          if (timer.hasElapsed(IntakeConstants.shootingAgitateReliefSeconds.getAsDouble())
              || inputs.slapdownDown) {
            completedCurls[0]++;
            curlingIn[0] = true;
            timer.restart();
          }
        },
        () -> {
          timer.stop();
          timer.reset();
          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

    /**
   * Briefly backs the slapdown away from the hopper during shooting agitation.
   *
   * <p>The relief speed is based on the existing shooting curl speed so tuning the curl speed keeps
   * the pulse behavior proportional.
   */
  private void requestShootingAgitateRelief() {
    outputs.slapdownStatorCurrentLimit =
        IntakeConstants.shootingSlowStowStatorCurrentLimit.getAsDouble();

    if (inputs.slapdownDown) {
      stopSlapdown();
      isSlapdownStopped = true;
      return;
    }

    double reliefSpeed =
        -IntakeConstants.shootingSlowStowSpeed.getAsDouble()
            * MathUtil.clamp(
                IntakeConstants.shootingAgitateReliefSpeedScalar.getAsDouble(), 0.0, 1.0);

    requestSlapdownSpeed(reliefSpeed, true);
  }

  /**
   * Slowly curls the slapdown inward while shooting using a tunable constant speed.
   *
   * <p>This is different from {@link #stowWhileShooting()}, which commands a position. This command
   * keeps the slapdown moving inward slowly at a tunable speed and uses a lower tunable stator
   * current limit for protection.
   *
   * <p>The command stops the slapdown once it reaches the shooting stop region, reaches the upper
   * limit sensor, or hits the current stop threshold.
   *
   * @return command that slowly curls the intake inward while shooting
   */
  public Command curlInWhileShootingSlowSpeed() {
    return runEnd(
        () -> {
          setRequestedRollerSpeed(0.8);
          requestSlapdownSlowStowSpeed();
        },
        () -> {
          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

  /**
   * Slowly curls the slapdown inward while shooting by ramping the requested position.
   *
   * <p>This is different from {@link #curlInWhileShootingSlowSpeed()}, which uses constant open-loop
   * speed. This command keeps closed-loop position control active, but moves the requested position
   * gradually from the current slapdown position toward {@link IntakeConstants#SHOOTING_STOP}.
   *
   * @return command that slowly ramps the slapdown to the shooting stop position
   */
  public Command curlInWhileShootingPositionRamp() {
    final Timer timer = new Timer();
    final double[] startPosition = new double[1];

    return runEnd(
        () -> {
          if (!timer.isRunning()) {
            startPosition[0] = inputs.slapdownPosition;
            timer.restart();
          }

          double progress =
              MathUtil.clamp(
                  timer.get() / IntakeConstants.shootingStowRampTimeSeconds.getAsDouble(),
                  0.0,
                  1.0);

          double desiredPosition =
              MathUtil.interpolate(startPosition[0], IntakeConstants.SHOOTING_STOP, progress);

          setRequestedRollerSpeed(0.0);
          requestSlapdownPosition(desiredPosition, SlapdownModeState.BUMP_POSITION, false);
        },
        () -> {
          timer.stop();
          timer.reset();
          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

  /**
   * Runs the intake rollers inward.
   *
   * @return command that runs the rollers until interrupted
   */
  public Command runRoller() {
    return runEnd(
        () -> setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED),
        () -> setRequestedRollerSpeed(0.0));
  }

  /**
   * Stops the rollers and slapdown continuously.
   *
   * <p>This is intended to be the intake default command.
   *
   * @return command that keeps the intake stopped
   */
  public Command stopIntake() {
    return run(
        () -> {
          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

  /**
   * Stops only the intake rollers once.
   *
   * <p>This does not stop or change the slapdown.
   *
   * @return instant command that sets requested roller speed to zero
   */
  public Command stopIntakeInstant() {
    return runOnce(() -> setRequestedRollerSpeed(0.0));
  }

  /**
   * Requests closed-loop slapdown position control.
   *
   * <p>This helper updates the desired slapdown position, selects the RobotState slapdown mode, and
   * controls whether the periodic current-spike stop logic is allowed to stop the motion.
   *
   * <p>The normal slapdown current limit is restored because position commands should not inherit
   * the lower current limit used by slow shooting stow.
   *
   * @param position requested slapdown position in current legacy slapdown units
   * @param slapdownMode RobotState mode used by the IO layer to choose the TalonFX control slot
   * @param stopOnCurrentSpike true when current-spike stop should end the slapdown motion
   */
  private void requestSlapdownPosition(
      double position, SlapdownModeState slapdownMode, boolean stopOnCurrentSpike) {
    isSlapdownStopped = false;
    restoreSlapdownCurrentLimit();
    outputs.desiredSlapdownPosition =
        MathUtil.clamp(position, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
    RobotState.setSlapdownMode(slapdownMode);
    stopSlapdownOnCurrentSpike = stopOnCurrentSpike;
  }

  /**
   * Requests open-loop slapdown speed control.
   *
   * <p>This is used by manual/debug motion and the slow constant-speed shooting stow command.
   *
   * @param speed requested slapdown open-loop output
   * @param stopOnCurrentSpike true when current-spike stop should end the slapdown motion
   */
  private void requestSlapdownSpeed(double speed, boolean stopOnCurrentSpike) {
    isSlapdownStopped = false;
    outputs.appliedSlapdownSpeed = speed;
    RobotState.setSlapdownMode(SlapdownModeState.SPEED);
    stopSlapdownOnCurrentSpike = stopOnCurrentSpike;
  }

  /**
   * Requests slow inward slapdown motion for shooting.
   *
   * <p>This uses constant low speed instead of a position ramp. The lower current limit is applied
   * through the IO layer.
   *
   * <p>If the slapdown is already at the upper limit or has already moved past the shooting stop
   * position, the slapdown is stopped instead of continuing inward.
   */
  private void requestSlapdownSlowStowSpeed() {
    outputs.slapdownStatorCurrentLimit =
        IntakeConstants.shootingSlowStowStatorCurrentLimit.getAsDouble();

    if (inputs.slapdownUp || inputs.slapdownPosition <= IntakeConstants.SHOOTING_STOP) {
      stopSlapdown();
      isSlapdownStopped = true;
      return;
    }

    requestSlapdownSpeed(IntakeConstants.shootingSlowStowSpeed.getAsDouble(), true);
  }

  /**
   * Stops slapdown motion and restores the normal slapdown current limit.
   *
   * <p>This does not change roller request state. Roller commands should call
   * {@link #setRequestedRollerSpeed(double)} separately.
   */
  private void stopSlapdown() {
    outputs.appliedSlapdownSpeed = 0.0;
    restoreSlapdownCurrentLimit();
    RobotState.setSlapdownMode(SlapdownModeState.OFF);
    stopSlapdownOnCurrentSpike = false;
  }

  /**
   * Stores the roller speed requested by the currently scheduled command.
   *
   * <p>The final applied roller output is still filtered by {@link #updateRollerOutput()} so the
   * slapdown position safety cutoff can stop the rollers when needed.
   *
   * @param speed requested roller output
   */
  private void setRequestedRollerSpeed(double speed) {
    requestedRollerSpeed = speed;
  }

  /** Resets all auto-boost state. */
  private void resetRollerBoostState() {
    rollerBoostActive = false;
    rollerWasRequested = false;
    rollerRequestStartTimestamp = 0.0;
    rollerHighCurrentStartTimestamp = Double.NaN;
    rollerBoostUntilTimestamp = 0.0;
  }

  /** Restores the normal slapdown stator current limit. */
  private void restoreSlapdownCurrentLimit() {
    outputs.slapdownStatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;
  }

  /**
   * Copies current tunable values into the output object used by the IO layer.
   *
   * <p>The IO layer owns deciding when to apply updated TalonFX configs. The subsystem simply keeps
   * the output object synchronized with the current tunable values.
   */
  private void updateTunableOutputs() {
    outputs.deployKP = IntakeConstants.kdeployP.getAsDouble();
    outputs.deployKI = IntakeConstants.kdeployI.getAsDouble();
    outputs.deployKD = IntakeConstants.kdeployD.getAsDouble();
    outputs.deployKG = IntakeConstants.kdeployG.getAsDouble();
    outputs.deployFF = IntakeConstants.kdeployFF.getAsDouble();

    outputs.stowKP = IntakeConstants.kstowP.getAsDouble();
    outputs.stowKI = IntakeConstants.kstowI.getAsDouble();
    outputs.stowKD = IntakeConstants.kstowD.getAsDouble();
    outputs.stowKG = IntakeConstants.kstowG.getAsDouble();
    outputs.stowFF = IntakeConstants.kstowFF.getAsDouble();
    outputs.stowMMAcceleration = IntakeConstants.kMMAcceleration.getAsDouble();
    outputs.stowMMJerk = IntakeConstants.kMMJerk.getAsDouble();

    outputs.stowFullKP = IntakeConstants.kstowFullP.getAsDouble();
    outputs.stowFullKI = IntakeConstants.kstowFullI.getAsDouble();
    outputs.stowFullKD = IntakeConstants.kstowFullD.getAsDouble();
    outputs.stowFullKG = IntakeConstants.kstowFullG.getAsDouble();
    outputs.stowFullFF = IntakeConstants.kstowFullFF.getAsDouble();
  }

  /**
   * Converts the requested roller speed into the final applied roller output.
   *
   * <p>Normal forward intake starts in duty-cycle mode. Auto boost can switch to torque-current mode
   * only after the roller has been requested for a short spin-up ignore period and high current has
   * persisted through a debounce window.
   */
  private void updateRollerOutput() {
    // if (inputs.slapdownPosition < IntakeConstants.ROLLER_STOP_CONSTRAINT && !overrideRollerSpeed) {
    //   outputs.appliedRollerSpeed = 0.0;
    //   RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
    //   resetRollerBoostState();
    //   logRollerBoostState(0.0, false);
    //   return;
    // }

    if (requestedRollerSpeed == 0.0) {
      outputs.appliedRollerSpeed = 0.0;
      RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
      resetRollerBoostState();
      logRollerBoostState(0.0, false);
      return;
    }

    // Preserve reverse/manual roller behavior. Boost is only for forward pickup.
    if (requestedRollerSpeed < 0.0) {
      outputs.appliedRollerSpeed = requestedRollerSpeed;
      RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
      resetRollerBoostState();
      logRollerBoostState(getMaxRollerStatorCurrent(), false);
      return;
    }

    double now = Timer.getTimestamp();
    double rollerCurrent = getMaxRollerStatorCurrent();

    if (!rollerWasRequested) {
      rollerWasRequested = true;
      rollerRequestStartTimestamp = now;
      rollerHighCurrentStartTimestamp = Double.NaN;
      rollerBoostActive = false;
      rollerBoostUntilTimestamp = 0.0;
    }

    boolean spinupComplete =
        now - rollerRequestStartTimestamp >= IntakeConstants.rollerBoostIgnoreSeconds.getAsDouble();

    if (spinupComplete && rollerCurrent >= IntakeConstants.rollerBoostEnterCurrent.getAsDouble()) {
      if (Double.isNaN(rollerHighCurrentStartTimestamp)) {
        rollerHighCurrentStartTimestamp = now;
      }

      if (now - rollerHighCurrentStartTimestamp
          >= IntakeConstants.rollerBoostDebounceSeconds.getAsDouble()) {
        rollerBoostActive = true;
        rollerBoostUntilTimestamp = now + IntakeConstants.rollerBoostHoldSeconds.getAsDouble();
      }
    } else {
      rollerHighCurrentStartTimestamp = Double.NaN;
    }

    if (rollerBoostActive
        && rollerCurrent <= IntakeConstants.rollerBoostExitCurrent.getAsDouble()
        && now >= rollerBoostUntilTimestamp) {
      rollerBoostActive = false;
      rollerHighCurrentStartTimestamp = Double.NaN;
    }

    if (rollerBoostActive) {
      outputs.appliedRollerSpeed = IntakeConstants.ROLLER_PICKUP_SPEED;
      RobotState.setRollerMode(RollerModeState.TORQUE_CURRENT);
    } else {
      outputs.appliedRollerSpeed = IntakeConstants.rollerNormalDuty.getAsDouble();
      RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
    }

    logRollerBoostState(rollerCurrent, spinupComplete);
  }

  /**
   * Logs auto-boost state for tuning and debugging.
   *
   * @param rollerCurrent max roller stator current
   * @param spinupComplete whether startup ignore time has elapsed
   */
  private void logRollerBoostState(double rollerCurrent, boolean spinupComplete) {
    Logger.recordOutput(kintakeTableKey + "RollerBoostActive", rollerBoostActive);
    Logger.recordOutput(kintakeTableKey + "RollerMaxStatorCurrent", rollerCurrent);
    Logger.recordOutput(kintakeTableKey + "RollerSpinupComplete", spinupComplete);
    Logger.recordOutput(kintakeTableKey + "RollerWasRequested", rollerWasRequested);
    Logger.recordOutput(kintakeTableKey + "RollerRequestStartTimestamp", rollerRequestStartTimestamp);
    Logger.recordOutput(
        kintakeTableKey + "RollerHighCurrentStartTimestamp", rollerHighCurrentStartTimestamp);
    Logger.recordOutput(kintakeTableKey + "RollerBoostUntilTimestamp", rollerBoostUntilTimestamp);
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostIgnoreSeconds",
        IntakeConstants.rollerBoostIgnoreSeconds.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostDebounceSeconds",
        IntakeConstants.rollerBoostDebounceSeconds.getAsDouble());
  }

  /**
   * Stops slapdown motion when a configured stop condition is reached.
   *
   * <p>For deploy/stow commands, the slapdown can stop from either current spike detection or the
   * requested limit sensor. Commands that do not request current-spike stopping still stop when the
   * requested hard-stop sensor becomes active.
   */
  private void stopSlapdownIfNeeded() {
    if (stopSlapdownOnCurrentSpike
        && inputs.slapdownSupplyCurrent >= IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD) {
      deployStop();
      return;
    }

    if (isAtRequestedLimit()) {
      deployStop();
    }
  }

  /**
   * Returns whether the slapdown has reached the hard-stop sensor for the requested full-travel
   * position.
   *
   * <p>This only checks full deploy and full stow positions. Intermediate positions, such as bump or
   * shooting stop, are not ended by this helper.
   *
   * @return true when the requested full-travel limit sensor is active
   */
  private boolean isAtRequestedLimit() {
    return (outputs.desiredSlapdownPosition == IntakeConstants.DOWN && inputs.slapdownDown)
        || (outputs.desiredSlapdownPosition == IntakeConstants.UP && inputs.slapdownUp);
  }

  /**
   * Ends the current slapdown deploy/stow motion.
   *
   * <p>This is used when the slapdown reaches a limit sensor or hits the current threshold. It stops
   * the motor, restores the normal current limit, clears current-spike stopping, and marks the
   * slapdown command wait condition as complete.
   */
  private void deployStop() {
    outputs.appliedSlapdownSpeed = 0.0;
    restoreSlapdownCurrentLimit();
    RobotState.setSlapdownMode(SlapdownModeState.OFF);
    stopSlapdownOnCurrentSpike = false;
    isSlapdownStopped = true;
  }

  /** Logs commanded intake state, tunables, and command helper state for debugging. */
  private void logOutputs() {
    Logger.recordOutput(kintakeTableKey + "SlapdownMode", RobotState.getSlapdownMode().toString());
    Logger.recordOutput(kintakeTableKey + "RollerMode", RobotState.getRollerMode().toString());
    Logger.recordOutput(kintakeTableKey + "RequestedRollerSpeed", requestedRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "AppliedRollerSpeed", outputs.appliedRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "AppliedSlapdownSpeed", outputs.appliedSlapdownSpeed);
    Logger.recordOutput(
        kintakeTableKey + "DesiredSlapdownPosition", outputs.desiredSlapdownPosition);
    Logger.recordOutput(
        kintakeTableKey + "SlapdownStatorCurrentLimit", outputs.slapdownStatorCurrentLimit);
    Logger.recordOutput(kintakeTableKey + "OverrideRollerSpeed", overrideRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "IsSlapdownStopped", isSlapdownStopped);
    Logger.recordOutput(kintakeTableKey + "StopSlapdownOnCurrentSpike", stopSlapdownOnCurrentSpike);

    Logger.recordOutput(
        kintakeTableKey + "RollerNormalDuty", IntakeConstants.rollerNormalDuty.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostTorqueCurrent",
        IntakeConstants.rollerBoostTorqueCurrent.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostEnterCurrent",
        IntakeConstants.rollerBoostEnterCurrent.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostExitCurrent",
        IntakeConstants.rollerBoostExitCurrent.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerBoostHoldSeconds",
        IntakeConstants.rollerBoostHoldSeconds.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "PrepareUnjamReverseSpeed",
        IntakeConstants.prepareUnjamReverseSpeed.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "PrepareUnjamReverseSeconds",
        IntakeConstants.prepareUnjamReverseSeconds.getAsDouble());

    Logger.recordOutput(
        kintakeTableKey + "ShootingSlowStowSpeed",
        IntakeConstants.shootingSlowStowSpeed.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "ShootingSlowStowStatorCurrentLimit",
        IntakeConstants.shootingSlowStowStatorCurrentLimit.getAsDouble());

    Logger.recordOutput(kdeployTableKey + "kP", outputs.deployKP);
    Logger.recordOutput(kdeployTableKey + "kI", outputs.deployKI);
    Logger.recordOutput(kdeployTableKey + "kD", outputs.deployKD);
    Logger.recordOutput(kdeployTableKey + "kG", outputs.deployKG);
    Logger.recordOutput(kdeployTableKey + "kFF", outputs.deployFF);

    Logger.recordOutput(kstowTableKey + "kP", outputs.stowKP);
    Logger.recordOutput(kstowTableKey + "kI", outputs.stowKI);
    Logger.recordOutput(kstowTableKey + "kD", outputs.stowKD);
    Logger.recordOutput(kstowTableKey + "kG", outputs.stowKG);
    Logger.recordOutput(kstowTableKey + "kFF", outputs.stowFF);
    Logger.recordOutput(kstowTableKey + "MMAcceleration", outputs.stowMMAcceleration);
    Logger.recordOutput(kstowTableKey + "MMJerk", outputs.stowMMJerk);

    Logger.recordOutput(kstowFullTableKey + "kP", outputs.stowFullKP);
    Logger.recordOutput(kstowFullTableKey + "kI", outputs.stowFullKI);
    Logger.recordOutput(kstowFullTableKey + "kD", outputs.stowFullKD);
    Logger.recordOutput(kstowFullTableKey + "kG", outputs.stowFullKG);
    Logger.recordOutput(kstowFullTableKey + "kFF", outputs.stowFullFF);

        Logger.recordOutput(
        kintakeTableKey + "ShootingAgitateCurlCount",
        IntakeConstants.shootingAgitateCurlCount.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "ShootingAgitateCurlSeconds",
        IntakeConstants.shootingAgitateCurlSeconds.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "ShootingAgitateReliefSeconds",
        IntakeConstants.shootingAgitateReliefSeconds.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "ShootingAgitateReliefSpeedScalar",
        IntakeConstants.shootingAgitateReliefSpeedScalar.getAsDouble());
  }

  /**
   * Manually sets the slapdown target angle.
   *
   * <p>This method only updates the desired position. It does not change the RobotState slapdown
   * mode, so it is kept for debugging only.
   *
   * @param angle requested slapdown angle
   */
  @Deprecated
  public void slapdownToPosition(double angle) {
    manualAngle = angle;
    outputs.desiredSlapdownPosition =
        MathUtil.clamp(manualAngle, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
  }

  /**
   * Runs the slapdown using a tunable open-loop speed.
   *
   * @return command that runs the slapdown at the tunable deploy speed until interrupted
   */
  public Command deployWithSpeed() {
    return runEnd(
        () -> requestSlapdownSpeed(IntakeConstants.deploySpeed.getAsDouble(), true),
        this::stopSlapdown);
  }

  /**
   * Runs the intake rollers without requiring the intake subsystem.
   *
   * <p>This is used by command groups that need to deploy the slapdown and run the rollers at the
   * same time.
   *
   * @return command that requests roller pickup speed without subsystem requirements
   */
  public Command runRollerWithoutRequirements() {
    return Commands.runEnd(
        () -> setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED),
        () -> setRequestedRollerSpeed(0.0));
  }

  /**
   * Briefly reverses the rollers to clear a pinch, then runs the rollers forward.
   *
   * <p>This command intentionally has no intake subsystem requirement so it can run alongside
   * {@link #deploy()}. During the reverse pulse, the roller safety override is enabled so the
   * rollers can move even while the slapdown is still above the normal roller cutoff position.
   *
   * @return command that unjams the rollers during prepare-intake, then runs them forward
   */
  public Command runRollerWithPrepareUnjamWithoutRequirements() {
    return Commands.defer(
        () ->
            Commands.sequence(
                runRollerReverseWithOverrideWithoutRequirements()
                    .until(() -> inputs.slapdownDown)
                    .withTimeout(
                        Math.max(
                            0.0, IntakeConstants.prepareUnjamReverseSeconds.getAsDouble())),
                runRollerWithoutRequirements()),
        Set.of());
  }

  /**
   * Runs the rollers in reverse while bypassing the slapdown position safety cutoff.
   *
   * @return command that reverse-runs the rollers until interrupted
   */
  private Command runRollerReverseWithOverrideWithoutRequirements() {
    return Commands.runEnd(
        () -> {
          setRequestedRollerSpeed(
              -Math.abs(IntakeConstants.prepareUnjamReverseSpeed.getAsDouble()));
          overrideRollerSpeed = true;
        },
        () -> {
          setRequestedRollerSpeed(0.0);
          overrideRollerSpeed = false;
        });
  }

  /**
   * Runs the rollers while bypassing the slapdown position safety cutoff.
   *
   * @return command that runs the rollers with the safety override enabled
   */
  public Command overrideRollerSpeedCommand() {
    return runEnd(
        () -> {
          setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED);
          overrideRollerSpeed = true;
        },
        () -> {
          setRequestedRollerSpeed(0.0);
          overrideRollerSpeed = false;
        });
  }

  /**
   * Reverses the intake rollers.
   *
   * <p>This is not currently bound in RobotContainer, but is kept as a debug/manual command.
   *
   * @return command that reverses the rollers until interrupted
   */
  @Deprecated
  public Command reverseRoller() {
    return runEnd(
        () -> setRequestedRollerSpeed(IntakeConstants.ROLLER_REVERSE_SPEED),
        () -> setRequestedRollerSpeed(0.0));
  }
}