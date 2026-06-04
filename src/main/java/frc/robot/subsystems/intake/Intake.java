package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kdeployTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowFullTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowTableKey;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LoopProfiler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.RollerModeState;
import frc.robot.RobotState.SlapdownModeState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  /** Roller control mode used only by the autonomous Prepare Intake latch. */
  public enum AutoPrepareRollerMode {
    DUTY_CYCLE,
    VOLTAGE,
    VELOCITY_DUTY_CYCLE,
    VELOCITY_VOLTAGE,
    VELOCITY_TORQUE_CURRENT_FOC
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();
  private final SendableChooser<AutoPrepareRollerMode> autoPrepareRollerModeChooser =
      new SendableChooser<>();
  private final LoggedDashboardChooser<AutoPrepareRollerMode> loggedAutoPrepareRollerModeChooser;

  private static final LoggedTunableNumber slowLogPeriodLoops =
      new LoggedTunableNumber(
          kintakeTableKey + "Logging/SlowPeriodLoops", 10.0, Constants.Tuning.INTAKE);

  private int slowLogCounter = 0;

  private boolean stopSlapdownOnCurrentSpike = false;
  private boolean isSlapdownStopped = true;
  private boolean deployHoldDownAssistEnabled = false;
  private boolean driverIntakeHeld = false;
  private boolean rollerBoostActive = false;
  private boolean autoPrepareIntakeRequested = false;
  private boolean autoPrepareIntakeLatched = false;
  private boolean rollerReverseOverride = false;
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
    configureAutoPrepareRollerModeChooser();
    loggedAutoPrepareRollerModeChooser =
        new LoggedDashboardChooser<>(
            kintakeTableKey + "Auto Prepare Roller Mode", autoPrepareRollerModeChooser);
  }

  /** Configures the roller control mode chooser used only by autonomous Prepare Intake. */
  private void configureAutoPrepareRollerModeChooser() {
    autoPrepareRollerModeChooser.setDefaultOption(
        "Velocity Torque Current FOC", AutoPrepareRollerMode.VELOCITY_TORQUE_CURRENT_FOC);
    autoPrepareRollerModeChooser.addOption(
        "Voltage", AutoPrepareRollerMode.VOLTAGE);
    autoPrepareRollerModeChooser.addOption(
        "Duty Cycle", AutoPrepareRollerMode.DUTY_CYCLE);
    autoPrepareRollerModeChooser.addOption(
        "Velocity Duty Cycle", AutoPrepareRollerMode.VELOCITY_DUTY_CYCLE);
    autoPrepareRollerModeChooser.addOption(
        "Velocity Voltage", AutoPrepareRollerMode.VELOCITY_VOLTAGE);
  }

  /** Returns the selected autonomous Prepare Intake roller mode. */
  private AutoPrepareRollerMode getAutoPrepareRollerMode() {
    AutoPrepareRollerMode selectedMode = loggedAutoPrepareRollerModeChooser.get();
    return selectedMode != null ? selectedMode : AutoPrepareRollerMode.VOLTAGE;
  }

  /**
   * Returns whether the autonomous Prepare Intake roller chooser should control the rollers.
   *
   * <p>During a real match this is only active for the autonomous Prepare Intake latch. During
   * tuning, the same path can be tested in teleop by holding the normal intake button.
   */
  private boolean shouldUsePrepareRollerMode() {
    return (DriverStation.isAutonomousEnabled() && autoPrepareIntakeLatched)
        || (Constants.tuningMode && DriverStation.isTeleopEnabled() && driverIntakeHeld);
  }

  /**
   * Updates intake inputs, updates tunable outputs, handles latched auto requests, applies requested
   * outputs, and stops the slapdown when a configured stop condition is reached.
   */
  @Override
  public void periodic() {
    LoopProfiler.run("Intake/UpdateInputs", () -> io.updateInputs(inputs));
    Logger.processInputs("Intake", inputs);

    boolean logSlowOutputs = shouldLogSlowIntakeOutputs();

    LoopProfiler.run("Intake/UpdateTunableOutputs", this::updateTunableOutputs);
    handleAutoPrepareIntakeRequest();
    LoopProfiler.run("Intake/UpdateRollerOutput", this::updateRollerOutput);
    LoopProfiler.run("Intake/UpdateDeployHoldDownAssist", this::updateDeployHoldDownAssist);
    logOutputs(logSlowOutputs);

    LoopProfiler.run("Intake/ApplyOutputs", () -> io.applyOutputs(outputs));

    LoopProfiler.run("Intake/StopSlapdownIfNeeded", this::stopSlapdownIfNeeded);
  }

  /** Returns true when slow-changing intake outputs should be logged this loop. */
  private boolean shouldLogSlowIntakeOutputs() {
    int periodLoops = Math.max(1, (int) Math.round(slowLogPeriodLoops.getAsDouble()));

    slowLogCounter++;
    if (slowLogCounter < periodLoops) {
      return false;
    }

    slowLogCounter = 0;
    return true;
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
   * <p>The command first moves to the down position normally. Once the deploy stop condition is
   * reached, it applies a short low-current force-down pulse to settle the intake farther down
   * without stalling the motor indefinitely. After that, hold-down assist remains enabled while
   * intaking so the slapdown can be nudged back down if it lifts.
   *
   * @return command that deploys the slapdown
   */
  public Command deploy() {
    return runOnce(() -> requestDeployWithHoldDownAssist())
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
            () ->
                requestSlapdownPosition(
                    IntakeConstants.UP, SlapdownModeState.STOW_POSITION, true))
        .andThen(new WaitUntilCommand(() -> isSlapdownStopped));
  }

  /**
   * Deploys the slapdown and then runs the rollers.
   *
   * <p>This intentionally does not wait for the down sensor after deploy completes. The down sensor
   * has enough range that small intake movement can make it flicker, but that should not stop the
   * rollers once the deploy command has completed.
   *
   * @return command that deploys the intake and then runs the rollers
   */
  public Command deployAndRunRoller() {
    return Commands.sequence(
            runOnce(
                () -> {
                  driverIntakeHeld = true;
                  requestDeployWithHoldDownAssist();
                }),
            run(
                () -> {
                  driverIntakeHeld = true;
                  setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED);
                }))
        .finallyDo(
            interrupted -> {
              driverIntakeHeld = false;
              setRequestedRollerSpeed(0.0);
              stopSlapdownHoldDownTorque();
            });
  }


  /**
   * Moves the slapdown to the bump/intermediate position.
   *
   * @return command that holds the bump position while scheduled
   */
  public Command stowBump() {
    return runEnd(
        () -> {
          driverIntakeHeld = false;
          clearAutoPrepareIntakeLatch();
          requestSlapdownPosition(IntakeConstants.BUMP, SlapdownModeState.BUMP_POSITION, false);
        },
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
          clearAutoPrepareIntakeLatch();
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
          clearAutoPrepareIntakeLatch();

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
          clearAutoPrepareIntakeLatch();
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
          clearAutoPrepareIntakeLatch();

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

  /** Requests autonomous intake preparation and finishes immediately. */
  public Command prepareIntakeInstant() {
    return runOnce(() -> autoPrepareIntakeRequested = true);
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
          if (DriverStation.isAutonomousEnabled() && autoPrepareIntakeLatched) {
            setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED);
            return;
          }

          setRequestedRollerSpeed(0.0);
          stopSlapdown();
        });
  }

  /** Clears autonomous intake request state. */
  private void clearAutoPrepareIntakeLatch() {
    autoPrepareIntakeRequested = false;
    autoPrepareIntakeLatched = false;
  }

  /**
   * Stops only the autonomous intake latch and rollers once.
   *
   * <p>This also disables deploy hold-down assist because the intake is no longer intentionally
   * running.
   *
   * @return instant command that clears autonomous intake state
   */
  public Command stopIntakeInstant() {
    return runOnce(
        () -> {
          driverIntakeHeld = false;
          clearAutoPrepareIntakeLatch();
          disableDeployHoldDownAssist();
          setRequestedRollerSpeed(0.0);
        });
  }

  /** Handles the one-cycle request created by the autonomous Prepare Intake named command. */
  private void handleAutoPrepareIntakeRequest() {
    if (!autoPrepareIntakeRequested) {
      return;
    }

    autoPrepareIntakeRequested = false;

    if (!DriverStation.isAutonomousEnabled()) {
      return;
    }

    autoPrepareIntakeLatched = true;
    requestDeployWithHoldDownAssist();
    setRequestedRollerSpeed(IntakeConstants.ROLLER_PICKUP_SPEED);
  }

  /** Starts normal deploy motion and enables deploy hold-down assist. */
  private void requestDeployWithHoldDownAssist() {
    deployHoldDownAssistEnabled = true;
    stopSlapdownHoldDownTorque();

    requestSlapdownPosition(IntakeConstants.DOWN, SlapdownModeState.DEPLOY_POSITION, true);
  }

  private boolean isFullTravelPositionMode() {
  return RobotState.getSlapdownMode() == SlapdownModeState.DEPLOY_POSITION
      || RobotState.getSlapdownMode() == SlapdownModeState.STOW_POSITION;
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
    if (slapdownMode != SlapdownModeState.DEPLOY_POSITION) {
      disableDeployHoldDownAssist();
    }

    isSlapdownStopped = false;
    restoreSlapdownCurrentLimit();
    outputs.appliedSlapdownSpeed = 0.0;
    outputs.appliedSlapdownTorqueCurrent = 0.0;
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
    disableDeployHoldDownAssist();

    isSlapdownStopped = false;
    outputs.appliedSlapdownSpeed = speed;
    outputs.appliedSlapdownTorqueCurrent = 0.0;
    RobotState.setSlapdownMode(SlapdownModeState.SPEED);
    stopSlapdownOnCurrentSpike = stopOnCurrentSpike;
  }

  /** Requests constant torque-current downforce from the slapdown motor. */
  private void requestSlapdownTorqueCurrent(double torqueCurrent) {
    isSlapdownStopped = false;
    outputs.appliedSlapdownSpeed = 0.0;
    outputs.appliedSlapdownTorqueCurrent = torqueCurrent;
    RobotState.setSlapdownMode(SlapdownModeState.TORQUE_CURRENT);
    stopSlapdownOnCurrentSpike = false;
  }

  /**
   * Applies constant downforce while the intake has deployed and the rollers are intaking.
   */
  private void updateDeployHoldDownAssist() {
    if (!deployHoldDownAssistEnabled) {
      return;
    }

    boolean holdDownRequested = driverIntakeHeld || autoPrepareIntakeLatched;

    if (!holdDownRequested) {
      stopSlapdownHoldDownTorque();
      return;
    }

    SlapdownModeState slapdownMode = RobotState.getSlapdownMode();

    if (slapdownMode == SlapdownModeState.STOW_POSITION
        || slapdownMode == SlapdownModeState.BUMP_POSITION
        || slapdownMode == SlapdownModeState.STOW_WHILE_SHOOTING
        || slapdownMode == SlapdownModeState.SPEED) {
      return;
    }

    requestSlapdownTorqueCurrent(IntakeConstants.slapdownHoldDownTorqueCurrent.getAsDouble());
  }


  /** Stops hold-down torque without disabling future hold-down assist. */
  private void stopSlapdownHoldDownTorque() {
    outputs.appliedSlapdownTorqueCurrent = 0.0;

    if (RobotState.getSlapdownMode() == SlapdownModeState.TORQUE_CURRENT) {
      RobotState.setSlapdownMode(SlapdownModeState.OFF);
      isSlapdownStopped = true;
    }
  }

  /** Disables deploy hold-down assist and clears any active hold-down torque. */
  private void disableDeployHoldDownAssist() {
    deployHoldDownAssistEnabled = false;
    stopSlapdownHoldDownTorque();
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
    disableDeployHoldDownAssist();
    outputs.appliedSlapdownSpeed = 0.0;
    outputs.appliedSlapdownTorqueCurrent = 0.0;
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

    outputs.rollerVelocityDutyKP = IntakeConstants.rollerVelocityDutyKP.getAsDouble();
    outputs.rollerVelocityDutyKI = IntakeConstants.rollerVelocityDutyKI.getAsDouble();
    outputs.rollerVelocityDutyKD = IntakeConstants.rollerVelocityDutyKD.getAsDouble();
    outputs.rollerVelocityDutyKS = IntakeConstants.rollerVelocityDutyKS.getAsDouble();
    outputs.rollerVelocityDutyKV = IntakeConstants.rollerVelocityDutyKV.getAsDouble();
    outputs.rollerVelocityDutyKA = IntakeConstants.rollerVelocityDutyKA.getAsDouble();

    outputs.rollerVelocityVoltageKP = IntakeConstants.rollerVelocityVoltageKP.getAsDouble();
    outputs.rollerVelocityVoltageKI = IntakeConstants.rollerVelocityVoltageKI.getAsDouble();
    outputs.rollerVelocityVoltageKD = IntakeConstants.rollerVelocityVoltageKD.getAsDouble();
    outputs.rollerVelocityVoltageKS = IntakeConstants.rollerVelocityVoltageKS.getAsDouble();
    outputs.rollerVelocityVoltageKV = IntakeConstants.rollerVelocityVoltageKV.getAsDouble();
    outputs.rollerVelocityVoltageKA = IntakeConstants.rollerVelocityVoltageKA.getAsDouble();

    outputs.rollerVelocityTorqueKP = IntakeConstants.rollerVelocityTorqueKP.getAsDouble();
    outputs.rollerVelocityTorqueKI = IntakeConstants.rollerVelocityTorqueKI.getAsDouble();
    outputs.rollerVelocityTorqueKD = IntakeConstants.rollerVelocityTorqueKD.getAsDouble();
    outputs.rollerVelocityTorqueKS = IntakeConstants.rollerVelocityTorqueKS.getAsDouble();
    outputs.rollerVelocityTorqueKV = IntakeConstants.rollerVelocityTorqueKV.getAsDouble();
    outputs.rollerVelocityTorqueKA = IntakeConstants.rollerVelocityTorqueKA.getAsDouble();
  }

  /**
   * Converts the requested roller intent into the final applied roller output.
   *
   * <p>Autonomous Prepare Intake uses the selected chooser mode and bypasses roller boost logic.
   * Teleop forward intake still uses torque-current mode with boost. Reverse override/manual roller
   * commands still use duty cycle.
   */
  private void updateRollerOutput() {
    // REFACTOR: If we restore the slapdown-position roller safety cutoff, re-add it here instead
    // of spreading safety checks into the individual roller commands.

    if (rollerReverseOverride) {
      outputs.appliedRollerSpeed = -Math.abs(IntakeConstants.ROLLER_REVERSE_SPEED);
      outputs.appliedRollerVoltage = 0.0;
      outputs.appliedRollerVelocityRPS = 0.0;
      RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
      resetRollerBoostState();
      logRollerBoostState(getMaxRollerStatorCurrent(), false);
      return;
    }

    if (shouldUsePrepareRollerMode()) {
      updateAutoPrepareRollerOutput();
      return;
    }

    outputs.appliedRollerVoltage = 0.0;
    outputs.appliedRollerVelocityRPS = 0.0;

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

    outputs.appliedRollerSpeed =
        rollerBoostActive
            ? IntakeConstants.rollerBoostTorqueCurrent.getAsDouble()
            : IntakeConstants.rollerNormalTorqueCurrent.getAsDouble();
    RobotState.setRollerMode(RollerModeState.TORQUE_CURRENT);

    logRollerBoostState(rollerCurrent, spinupComplete);
  }

  /** Applies the selected roller mode for autonomous Prepare Intake or teleop tuning. */
  private void updateAutoPrepareRollerOutput() {
    resetRollerBoostState();
    logRollerBoostState(getMaxRollerStatorCurrent(), false);

    AutoPrepareRollerMode selectedMode = getAutoPrepareRollerMode();

    switch (selectedMode) {
      case VELOCITY_DUTY_CYCLE:
        outputs.appliedRollerSpeed = 0.0;
        outputs.appliedRollerVoltage = 0.0;
        outputs.appliedRollerVelocityRPS = IntakeConstants.autoPrepareRollerVelocityRPS.getAsDouble();
        RobotState.setRollerMode(RollerModeState.VELOCITY_DUTY_CYCLE);
        break;

      case VELOCITY_VOLTAGE:
        outputs.appliedRollerSpeed = 0.0;
        outputs.appliedRollerVoltage = 0.0;
        outputs.appliedRollerVelocityRPS = IntakeConstants.autoPrepareRollerVelocityRPS.getAsDouble();
        RobotState.setRollerMode(RollerModeState.VELOCITY_VOLTAGE);
        break;

      case VELOCITY_TORQUE_CURRENT_FOC:
        outputs.appliedRollerSpeed = 0.0;
        outputs.appliedRollerVoltage = 0.0;
        outputs.appliedRollerVelocityRPS = IntakeConstants.autoPrepareRollerVelocityRPS.getAsDouble();
        RobotState.setRollerMode(RollerModeState.VELOCITY_TORQUE_CURRENT_FOC);
        break;

      case VOLTAGE:
        outputs.appliedRollerSpeed = 0.0;
        outputs.appliedRollerVoltage = IntakeConstants.autoPrepareRollerVoltage.getAsDouble();
        outputs.appliedRollerVelocityRPS = 0.0;
        RobotState.setRollerMode(RollerModeState.VOLTAGE);
        break;

      case DUTY_CYCLE:
      default:
        outputs.appliedRollerVoltage = 0.0;
        outputs.appliedRollerVelocityRPS = 0.0;
        outputs.appliedRollerSpeed = IntakeConstants.autoPrepareRollerDutyCycle.getAsDouble();
        RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
        break;
    }
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
   * requested limit sensor. For deploy hold-down assist, the pulse stops from timeout or supply
   * current. It intentionally does not stop just because the down sensor is true because that sensor
   * has a wide range of motion.
   */
  private void stopSlapdownIfNeeded() {
    if (stopSlapdownOnCurrentSpike
        && inputs.slapdownSupplyCurrent >= IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD) {
      deployStop();
      return;
    }

    if (isFullTravelPositionMode() && isAtRequestedLimit()) {
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
    outputs.appliedSlapdownTorqueCurrent = 0.0;
    restoreSlapdownCurrentLimit();
    RobotState.setSlapdownMode(SlapdownModeState.OFF);
    stopSlapdownOnCurrentSpike = false;
    isSlapdownStopped = true;
  }

  /** Logs commanded intake state, tunables, and command helper state for debugging. */
  private void logOutputs(boolean logSlowOutputs) {
    Logger.recordOutput(kintakeTableKey + "SlapdownMode", RobotState.getSlapdownMode().toString());
    Logger.recordOutput(kintakeTableKey + "RollerMode", RobotState.getRollerMode().toString());
    Logger.recordOutput(kintakeTableKey + "RequestedRollerSpeed", requestedRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "AppliedRollerSpeed", outputs.appliedRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "AppliedSlapdownSpeed", outputs.appliedSlapdownSpeed);
    Logger.recordOutput(
        kintakeTableKey + "AppliedSlapdownTorqueCurrent", outputs.appliedSlapdownTorqueCurrent);
    Logger.recordOutput(
        kintakeTableKey + "DesiredSlapdownPosition", outputs.desiredSlapdownPosition);
    Logger.recordOutput(
        kintakeTableKey + "SlapdownStatorCurrentLimit", outputs.slapdownStatorCurrentLimit);
    Logger.recordOutput(kintakeTableKey + "IsSlapdownStopped", isSlapdownStopped);
    Logger.recordOutput(kintakeTableKey + "StopSlapdownOnCurrentSpike", stopSlapdownOnCurrentSpike);
    Logger.recordOutput(
        kintakeTableKey + "DeployHoldDownAssistEnabled", deployHoldDownAssistEnabled);
    Logger.recordOutput(
        kintakeTableKey + "DeployHoldDownActive",
        RobotState.getSlapdownMode() == SlapdownModeState.TORQUE_CURRENT);
    Logger.recordOutput(kintakeTableKey + "AutoPrepareIntakeRequested", autoPrepareIntakeRequested);
    Logger.recordOutput(kintakeTableKey + "AutoPrepareIntakeLatched", autoPrepareIntakeLatched);
    Logger.recordOutput(kintakeTableKey + "RollerReverseOverride", rollerReverseOverride);
    Logger.recordOutput(
        kintakeTableKey + "SlapdownHoldDownTorqueCurrent",
        IntakeConstants.slapdownHoldDownTorqueCurrent.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "SlapdownLowerSupplyCurrentLimit",
        IntakeConstants.SLAPDOWN_LOWER_SUPPLY_CURRENT_LIMIT);
    Logger.recordOutput(
        kintakeTableKey + "SlapdownLowerSupplyCurrentTime",
        IntakeConstants.SLAPDOWN_LOWER_SUPPLY_CURRENT_TIME);

    if (!logSlowOutputs) {
      return;
    }

    Logger.recordOutput(
        kintakeTableKey + "RollerNormalTorqueCurrent",
        IntakeConstants.rollerNormalTorqueCurrent.getAsDouble());
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
        kintakeTableKey + "AutoPrepareRollerMode",
        getAutoPrepareRollerMode().toString());
    Logger.recordOutput(
        kintakeTableKey + "UsingPrepareRollerMode",
        shouldUsePrepareRollerMode());
    Logger.recordOutput(
        kintakeTableKey + "AppliedRollerVoltage",
        outputs.appliedRollerVoltage);
    Logger.recordOutput(
        kintakeTableKey + "AppliedRollerVelocityRPS",
        outputs.appliedRollerVelocityRPS);
    Logger.recordOutput(
        kintakeTableKey + "AutoPrepareRollerDutyCycle",
        IntakeConstants.autoPrepareRollerDutyCycle.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "AutoPrepareRollerVoltage",
        IntakeConstants.autoPrepareRollerVoltage.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "AutoPrepareRollerVelocityRPS",
        IntakeConstants.autoPrepareRollerVelocityRPS.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerVelocityDuty/kP",
        IntakeConstants.rollerVelocityDutyKP.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerVelocityVoltage/kP",
        IntakeConstants.rollerVelocityVoltageKP.getAsDouble());
    Logger.recordOutput(
        kintakeTableKey + "RollerVelocityTorque/kP",
        IntakeConstants.rollerVelocityTorqueKP.getAsDouble());

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

  public void useAutoRollerCurrentLimits() {
    io.useAutoRollerCurrentLimits();
  }

  public void useTeleopRollerCurrentLimits() {
    io.useTeleopRollerCurrentLimits();
  }

  /**
   * Runs the rollers from the operator debug binding.
   *
   * <p>REFACTOR: This keeps the old method name so RobotContainer does not need to change yet. The
   * old slapdown-position safety override was removed because that safety cutoff is not active.
   *
   * @return command that runs the rollers until interrupted
   */
  public Command overrideRollerSpeedCommand() {
    return runRoller();
  }

  /**
   * Forces the intake rollers to reverse while held without interrupting the current intake state.
   *
   * <p>This command intentionally does not require the intake subsystem. It only sets a temporary
   * override flag, so releasing the button returns the rollers to whatever the subsystem would have
   * otherwise been doing, such as auto Prepare Intake, teleop intake, or stopped.
   *
   * @return command that forces reverse roller output while scheduled
   */
  public Command reverseRoller() {
    return Commands.startEnd(
        () -> rollerReverseOverride = true,
        () -> rollerReverseOverride = false);
  }

  public Command barfRollers() {
    return runEnd(
        () -> {
          driverIntakeHeld = false;
          clearAutoPrepareIntakeLatch();
          setRequestedRollerSpeed(IntakeConstants.ROLLER_BARF_SPEED);
        },
        () -> setRequestedRollerSpeed(0.0));
  }
}