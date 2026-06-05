package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Controls the shooter flywheel.
 *
 * <p>The shooter is commanded in RPM at the subsystem level. The IO layer converts RPM to the CTRE
 * rotations-per-second units required by the motor controllers.
 */
public class Shooter extends SubsystemBase {
  /** Idle target source used by the shooter default command. */
  public enum ShooterIdleMode {
    OFF,
    FIXED,
    DYNAMIC
  }

  private enum ShooterIdleZone {
    OWN_ALLIANCE,
    NEUTRAL,
    OPPONENT_ALLIANCE
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputsAutoLogged outputs = new ShooterIOOutputsAutoLogged();
  private final SendableChooser<ShooterIdleMode> idleModeChooser = new SendableChooser<>();
  private final LoggedDashboardChooser<ShooterIdleMode> loggedIdleModeChooser;

  private boolean hasSpeedTargetChanged = true;
  private boolean defaultShouldCoast = true;
  private boolean defaultIdleEnabled = true;
  private double rampedIdleRPM = 0.0;
  private double requestedIdleRPM = 0.0;
  private double activeIdleRampUpRPMPerSec = 0.0;
  private double stallCommandActiveSinceTimestamp = Double.NaN;
  private double stallDetectedSinceTimestamp = Double.NaN;
  private double stallShutoffUntilTimestamp = 0.0;
  private boolean stallProtectionActive = false;
  private int slowLogCounter = 0;

  /**
   * Creates a shooter subsystem using the provided hardware implementation.
   *
   * @param io shooter hardware abstraction
   */
  public Shooter(ShooterIO io) {
    this.io = io;
    configureIdleModeChooser();
    loggedIdleModeChooser =
        new LoggedDashboardChooser<>(SHOOTER_TABLE_KEY + "Idle Mode", idleModeChooser);

    if (Constants.Tuning.SHOOTER) {
      SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);
    }
  }

  /** Configures the dashboard chooser used to select the default idle behavior. */
  private void configureIdleModeChooser() {
    idleModeChooser.setDefaultOption("Dynamic", ShooterIdleMode.DYNAMIC);
    idleModeChooser.addOption("Fixed", ShooterIdleMode.FIXED);
    idleModeChooser.addOption("Off", ShooterIdleMode.OFF);
  }

  /** Returns the idle mode selected on the dashboard chooser. */
  private ShooterIdleMode getSelectedIdleMode() {
    ShooterIdleMode selectedMode = loggedIdleModeChooser.get();
    return selectedMode != null ? selectedMode : ShooterIdleMode.DYNAMIC;
  }

  /** Returns the idle mode actually used by the robot. */
  private ShooterIdleMode getActiveIdleMode() {
    return DriverStation.isAutonomousEnabled() ? ShooterIdleMode.FIXED : getSelectedIdleMode();
  }

  /** Updates shooter inputs, tunables, readiness logs, and applies requested outputs. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    updateTunableOutputs();

    if (Constants.Tuning.SHOOTER) {
      io.tuneMotorConfigs(outputs);
    }

    applyLowCeilingLimitIfNeeded();
    updateStallProtection();
    logShooterState();

    io.applyOutputs(outputs);
  }

  /**
   * Default shooter behavior that safely returns the flywheel to idle.
   *
   * <p>When the flywheel is well above idle, the shooter output is turned off so the flywheel coasts
   * down naturally. Once the measured speed is close to idle, closed-loop idle control resumes.
   *
   * <p>The enter and exit thresholds intentionally use hysteresis so the shooter does not chatter
   * between OFF and IDLE near the idle RPM.
   *
   * @return default shooter command
   */
  public Command coastShooterDefaultCommand() {
    return run(
        () -> {
          if (!defaultIdleEnabled) {
            rampedIdleRPM = 0.0;
            requestedIdleRPM = 0.0;
            requestShooterOff();
            return;
          }

          requestedIdleRPM = getRequestedIdleRPM(getActiveIdleMode());
          double idleRPM = updateRampedIdleRPM(requestedIdleRPM);

          if (idleRPM <= ShooterConstants.idleMinCommandRPM.getAsDouble()) {
            requestShooterOff();
            return;
          }

          updateDefaultCoastState(idleRPM);

          if (defaultShouldCoast
              || getMeasuredShooterRPM()
                  > idleRPM + ShooterConstants.IDLE_COAST_EXIT_MARGIN_RPM) {
            requestShooterOff();
          } else {
            requestShooterVelocity(ShooterModeState.IDLE, idleRPM);
          }
        });
  }

  /**
   * Holds the shooter at the configured idle RPM.
   *
   * <p>This is kept as a direct command for testing, but the normal default command should be
   * {@link #coastShooterDefaultCommand()} so high-speed flywheel coast-down is protected.
   *
   * @return command that holds shooter idle speed while scheduled
   */
  public Command runIdleCommand() {
    return run(
        () -> {
          double idleRPM = updateRampedIdleRPM(getRequestedIdleRPM(getActiveIdleMode()));
          if (idleRPM <= ShooterConstants.idleMinCommandRPM.getAsDouble()) {
            requestShooterOff();
          } else {
            requestShooterVelocity(ShooterModeState.IDLE, idleRPM);
          }
        });
  }

  /** Requests idle shooter speed once and finishes immediately. */
  public Command idleShooterInstant() {
    return runOnce(
        () -> {
          enableDefaultIdle();
          requestedIdleRPM = getRequestedIdleRPM(getActiveIdleMode());
        });
  }

  /** Allows the default command to hold idle speed. */
  public void enableDefaultIdle() {
    defaultIdleEnabled = true;
  }

  /** Prevents the default command from idling the shooter. */
  public void disableDefaultIdle() {
    defaultIdleEnabled = false;
    defaultShouldCoast = true;
    requestShooterOff();
  }

  public Command enableDefaultIdleCommand() {
    return runOnce(this::enableDefaultIdle);
  }

  public Command disableDefaultIdleCommand() {
    return runOnce(this::disableDefaultIdle);
  }

  /**
   * Runs the shooter at the fixed tunable shooting RPM.
   *
   * @return command that runs the shooter at fixed RPM while scheduled
   */
  public Command runFixedSpeedCommand() {
    return run(
        () -> requestShooterVelocity(
            ShooterModeState.ON, ShooterConstants.shootFixedRPM.getAsDouble()));
  }

  /**
   * Runs the shooter at the calculated target RPM from {@link ShooterCalculation}.
   *
   * @return command that continuously updates the shooter target while scheduled
   */
  public Command runShooterTarget() {
    return run(
        () -> {
          var params = ShooterCalculation.getInstance().getParameters();
          double flywheelRPM =
              MathUtil.clamp(
                  params.flywheelSpeed(), 0.0, ShooterConstants.MAX_FLYWHEEL_CALCULATED_RPM);

          requestShooterVelocity(ShooterModeState.ON, flywheelRPM);

          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target/Passing", params.passing());
          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target/FlywheelRPM", params.flywheelSpeed());
          Logger.recordOutput(
              SHOOTER_TABLE_KEY + "Target/CommandedFlywheelRPM", outputs.desiredVelocityRPM);
        });
  }

  /**
   * Runs the shooter in open-loop duty-cycle mode.
   *
   * @param dutyCycle requested shooter duty cycle
   * @return command that runs the shooter at the requested duty cycle
   */
  public Command runShooterDutyCycle(double dutyCycle) {
    return run(
        () -> {
          setShooterMode(ShooterModeState.DUTYCYCLE);
          outputs.desiredVelocityRPM = 0.0;
          outputs.desiredDutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        });
  }

  /**
   * Stops shooter output and lets the flywheel coast.
   *
   * @return command that keeps the shooter off while scheduled
   */
  public Command stopAndCoastShooter() {
    return run(this::requestShooterOff);
  }

  /**
   * Returns whether the shooter is ready at the current target.
   *
   * <p>After a target change, this returns false once before allowing the raw velocity check to pass.
   *
   * @return true when the shooter is at the current target and the target has settled for one check
   */
  public BooleanSupplier isShooterAtVelocity() {
    return () -> {
      if (hasSpeedTargetChanged) {
        hasSpeedTargetChanged = false;
        return false;
      }

      return rawShooterAtCurrentTarget();
    };
  }

  /**
   * Returns whether the shooter is close enough to idle for closed-loop idle control.
   *
   * @return true when the measured shooter speed is within the idle coast exit threshold
   */
  public BooleanSupplier isShooterBelowCoastRPM() {
    return () ->
        getMeasuredShooterRPM()
            <= Math.max(ShooterConstants.fixedIdleRPM.getAsDouble(), rampedIdleRPM)
                + ShooterConstants.IDLE_COAST_EXIT_MARGIN_RPM;
  }

  /**
   * Updates whether the default command should coast or hold idle.
   *
   * <p>This small hysteresis state machine replaces the old timed {@code ConditionalCommand}.
   */
  private void updateDefaultCoastState(double idleRPM) {
    double measuredRPM = getMeasuredShooterRPM();

    double coastEnterRPM = idleRPM + ShooterConstants.IDLE_COAST_ENTER_MARGIN_RPM;
    double coastExitRPM = idleRPM + ShooterConstants.IDLE_COAST_EXIT_MARGIN_RPM;

    if (defaultShouldCoast && measuredRPM <= coastExitRPM) {
      defaultShouldCoast = false;
    } else if (!defaultShouldCoast && measuredRPM >= coastEnterRPM) {
      defaultShouldCoast = true;
    }
  }

  /** Returns the unramped idle target for the selected idle mode. */
  private double getRequestedIdleRPM(ShooterIdleMode idleMode) {
    switch (idleMode) {
      case OFF:
        return 0.0;

      case DYNAMIC:
        return getDynamicIdleRPM();

      case FIXED:
      default:
        return ShooterConstants.fixedIdleRPM.getAsDouble();
    }
  }

  /** Returns the dynamic idle target based on the robot's current field zone. */
  private double getDynamicIdleRPM() {
    return switch (getCurrentIdleZone()) {
      case OWN_ALLIANCE -> ShooterConstants.dynamicOwnAllianceZoneIdleRPM.getAsDouble();
      case NEUTRAL -> ShooterConstants.dynamicNeutralZoneIdleRPM.getAsDouble();
      case OPPONENT_ALLIANCE -> ShooterConstants.dynamicOpponentAllianceZoneIdleRPM.getAsDouble();
    };
  }

  /** Returns the robot's current zone, expressed from our alliance's side of the field. */
  private ShooterIdleZone getCurrentIdleZone() {
    double allianceRelativeX =
        AllianceFlipUtil.applyX(RobotState.getInstance().getRobotPoseField().getX());

    if (allianceRelativeX <= FieldConstants.LinesVertical.allianceZone) {
      return ShooterIdleZone.OWN_ALLIANCE;
    }

    if (allianceRelativeX >= FieldConstants.LinesVertical.oppAllianceZone) {
      return ShooterIdleZone.OPPONENT_ALLIANCE;
    }

    return ShooterIdleZone.NEUTRAL;
  }

  /** Slews the idle target up gently. Target decreases are handled by coasting down. */
  private double updateRampedIdleRPM(double targetRPM) {
    double clampedTargetRPM = MathUtil.clamp(targetRPM, 0.0, ShooterConstants.MAX_FLYWHEEL_RPM);

    if (clampedTargetRPM <= rampedIdleRPM) {
      rampedIdleRPM = clampedTargetRPM;
      activeIdleRampUpRPMPerSec = 0.0;
      return rampedIdleRPM;
    }

    double rampUpRPMPerSec = ShooterConstants.idleRampUpRPMPerSec.getAsDouble();
    double maxDelta = rampUpRPMPerSec * Constants.loopPeriodSecs;

    rampedIdleRPM = MathUtil.clamp(clampedTargetRPM, rampedIdleRPM, rampedIdleRPM + maxDelta);
    activeIdleRampUpRPMPerSec = rampUpRPMPerSec;

    return rampedIdleRPM;
  }

  private void updateStallProtection() {
    double now = Timer.getFPGATimestamp();

    if (!ShooterConstants.stallProtectionEnabled.getAsBoolean()) {
      resetStallProtection(false);
      return;
    }

    if (now < stallShutoffUntilTimestamp) {
      stallProtectionActive = true;
      stallDetectedSinceTimestamp = Double.NaN;
      requestShooterOff();
      return;
    }

    stallProtectionActive = false;

    if (!isShooterCommandedForStallDetection()) {
      resetStallProtection(false);
      return;
    }

    if (!Double.isFinite(stallCommandActiveSinceTimestamp)) {
      stallCommandActiveSinceTimestamp = now;
    }

    if (now - stallCommandActiveSinceTimestamp
        < ShooterConstants.stallStartupGraceSeconds.getAsDouble()) {
      stallDetectedSinceTimestamp = Double.NaN;
      return;
    }

    boolean stalled =
        getMeasuredShooterRPM() <= ShooterConstants.stallMaxMeasuredRPM.getAsDouble()
            && getMaxShooterStatorCurrent() >= ShooterConstants.stallStatorCurrentAmps.getAsDouble();

    if (!stalled) {
      stallDetectedSinceTimestamp = Double.NaN;
      return;
    }

    if (!Double.isFinite(stallDetectedSinceTimestamp)) {
      stallDetectedSinceTimestamp = now;
    }

    if (now - stallDetectedSinceTimestamp >= ShooterConstants.stallDebounceSeconds.getAsDouble()) {
      stallShutoffUntilTimestamp = now + ShooterConstants.stallShutoffSeconds.getAsDouble();
      stallProtectionActive = true;
      requestShooterOff();
    }
  }

  private boolean isShooterCommandedForStallDetection() {
    return (RobotState.getShooterMode() == ShooterModeState.ON
            || RobotState.getShooterMode() == ShooterModeState.IDLE)
        && outputs.desiredVelocityRPM >= ShooterConstants.stallMinCommandRPM.getAsDouble();
  }

  private double getMaxShooterStatorCurrent() {
    return Math.max(
        Math.max(inputs.rightLeaderStatorCurrent, inputs.rightFollowerStatorCurrent),
        Math.max(inputs.leftLeaderStatorCurrent, inputs.leftFollowerStatorCurrent));
  }

  private void resetStallProtection(boolean keepCooldown) {
    stallCommandActiveSinceTimestamp = Double.NaN;
    stallDetectedSinceTimestamp = Double.NaN;
    stallProtectionActive = false;

    if (!keepCooldown) {
      stallShutoffUntilTimestamp = 0.0;
    }
  }

  /**
   * Requests a closed-loop shooter velocity.
   *
   * @param mode shooter mode to use
   * @param rpm requested shooter speed in RPM
   */
  private void requestShooterVelocity(ShooterModeState mode, double rpm) {
    setShooterMode(mode);
    setDesiredVelocityRPM(rpm);
    outputs.desiredDutyCycle = 0.0;
  }

  /** Requests shooter OFF so the flywheel can coast naturally. */
  private void requestShooterOff() {
    setShooterMode(ShooterModeState.OFF);
    setDesiredVelocityRPM(0.0);
    outputs.desiredDutyCycle = 0.0;
  }

  /** Copies current tunable values into the output object used by the IO layer. */
  private void updateTunableOutputs() {
    outputs.useMotionMagic = ShooterConstants.useMotionMagic.getAsBoolean();
    outputs.kP = ShooterConstants.kP.getAsDouble();
    outputs.kI = ShooterConstants.kI.getAsDouble();
    outputs.kD = ShooterConstants.kD.getAsDouble();
    outputs.kS = ShooterConstants.kS.getAsDouble();
    outputs.kV = ShooterConstants.kV.getAsDouble();
    outputs.kA = ShooterConstants.kA.getAsDouble();
    outputs.kMMAcceleration = ShooterConstants.kMMAcceleration.getAsDouble();
    outputs.kMMShootAcceleration = ShooterConstants.kMMShootAcceleration.getAsDouble();
    outputs.kMMJerk = ShooterConstants.kMMJerk.getAsDouble();
  }

  /** Applies the low-ceiling RPM clamp when enabled. */
  private void applyLowCeilingLimitIfNeeded() {
    if (!ShooterConstants.isLowCeiling || RobotState.getShooterMode() != ShooterModeState.ON) {
      return;
    }

    outputs.desiredVelocityRPM =
        MathUtil.clamp(
            outputs.desiredVelocityRPM * ShooterConstants.FLYWHEEL_LOW_CEILING_SCALER,
            0.0,
            ShooterConstants.MAX_FLYWHEEL_LOW_CEILING_RPM);
  }

  /**
   * Updates shooter mode and marks the current speed target as changed when the mode changes.
   *
   * @param mode requested shooter mode
   */
  private void setShooterMode(ShooterModeState mode) {
    if (RobotState.getShooterMode() != mode) {
      hasSpeedTargetChanged = true;
    }

    RobotState.setShooterMode(mode);
  }

  /**
   * Updates the desired shooter velocity.
   *
   * @param rpm desired shooter speed in RPM
   */
  private void setDesiredVelocityRPM(double rpm) {
    if (Math.abs(rpm - outputs.desiredVelocityRPM)
        > ShooterConstants.flywheelUnderToleranceRPM.getAsDouble()) {
      hasSpeedTargetChanged = true;
    }

    outputs.desiredVelocityRPM = rpm;
  }

  /**
   * Returns the measured shooter RPM used for readiness and default idle/coast logic.
   *
   * @return measured right leader shooter speed in RPM
   */
  private double getMeasuredShooterRPM() {
    return inputs.rightLeaderVelocityRPM;
  }

  /**
   * Returns whether the shooter is at the currently requested target.
   *
   * @return true when measured speed is within tolerance of desired speed
   */
  private boolean rawShooterAtCurrentTarget() {
    double measuredRPM = getMeasuredShooterRPM();
    double errorRPM = outputs.desiredVelocityRPM - measuredRPM;
    double allowedErrorRPM =
        errorRPM >= 0.0
            ? ShooterConstants.flywheelUnderToleranceRPM.getAsDouble()
            : ShooterConstants.flywheelOverToleranceRPM.getAsDouble();

    return Math.abs(errorRPM) <= allowedErrorRPM;
  }

  /** Logs requested shooter state and readiness values. */
  private void logShooterState() {
    boolean rawAtCurrentTarget = rawShooterAtCurrentTarget();
    boolean slowThisLoop = shouldLogSlowShooterOutputs();

    // Keep these fast. These are useful during a shot.
    Logger.recordOutput(SHOOTER_TABLE_KEY + "ShooterMode", RobotState.getShooterMode().toString());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredVelocityRPM", outputs.desiredVelocityRPM);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "MeasuredVelocityRPM", getMeasuredShooterRPM());
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "VelocityErrorRPM",
        outputs.desiredVelocityRPM - getMeasuredShooterRPM());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "RawShooterAtCurrentTarget", rawAtCurrentTarget);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "IsShooterReadyFiltered",
        !hasSpeedTargetChanged && rawAtCurrentTarget);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "StallProtection/Active", stallProtectionActive);

    // Slow match/debug logs. Useful, but not needed at 50 Hz.
    if (slowThisLoop) {
      Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredDutyCycle", outputs.desiredDutyCycle);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "UseMotionMagic", outputs.useMotionMagic);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/Mode", getActiveIdleMode().toString());
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/SelectedMode", getSelectedIdleMode().toString());
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/RequestedRPM", requestedIdleRPM);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/RampedRPM", rampedIdleRPM);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/Zone", getCurrentIdleZone().toString());
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + "BelowCoastRPM", isShooterBelowCoastRPM().getAsBoolean());
      Logger.recordOutput(SHOOTER_TABLE_KEY + "HasSpeedTargetChanged", hasSpeedTargetChanged);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Default/ShouldCoast", defaultShouldCoast);
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + "StallProtection/MaxStatorCurrent", getMaxShooterStatorCurrent());
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + "StallProtection/ShutoffRemainingSeconds",
          Math.max(0.0, stallShutoffUntilTimestamp - Timer.getFPGATimestamp()));
    }

    // Tuning/config mirror logs only when shooter tuning is enabled.
    if (Constants.Tuning.SHOOTER && slowThisLoop) {
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + "Tolerance/UnderRPM",
          ShooterConstants.flywheelUnderToleranceRPM.getAsDouble());
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + "Tolerance/OverRPM",
          ShooterConstants.flywheelOverToleranceRPM.getAsDouble());
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/ActiveRampUpRPMPerSec", activeIdleRampUpRPMPerSec);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/UseMotionMagic", outputs.useMotionMagic);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kP", outputs.kP);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kI", outputs.kI);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kD", outputs.kD);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kS", outputs.kS);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kV", outputs.kV);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kA", outputs.kA);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/MMAcceleration", outputs.kMMAcceleration);
      Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/MMJerk", outputs.kMMJerk);
    }
  }

  /** Returns true at the slower logging cadence for rarely changing shooter outputs. */
  private boolean shouldLogSlowShooterOutputs() {
    int periodLoops =
        Math.max(1, (int) Math.round(ShooterConstants.slowLogPeriodLoops.getAsDouble()));

    slowLogCounter++;
    if (slowLogCounter < periodLoops) {
      return false;
    }

    slowLogCounter = 0;
    return true;
  }


  /** @return raw IO shooter-at-velocity value for debugging */
  public boolean getRawShooterAtVelocityForDebug() {
    return inputs.shooterAtVelocity;
  }

  /** @return whether the shooter target-change filter is currently blocking readiness */
  public boolean getHasSpeedTargetChangedForDebug() {
    return hasSpeedTargetChanged;
  }

  /** @return current desired shooter velocity in RPM */
  public double getDesiredVelocityRPMForDebug() {
    return outputs.desiredVelocityRPM;
  }
}