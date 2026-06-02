package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;
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
    PASSING,
    SHOOTING,
    DYNAMIC
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
    idleModeChooser.addOption("Passing", ShooterIdleMode.PASSING);
    idleModeChooser.addOption("Shooting", ShooterIdleMode.SHOOTING);
    idleModeChooser.addOption("Off", ShooterIdleMode.OFF);
  }

  /** Returns the selected shooter idle mode. */
  private ShooterIdleMode getSelectedIdleMode() {
    ShooterIdleMode selectedMode = loggedIdleModeChooser.get();
    return selectedMode != null ? selectedMode : ShooterIdleMode.DYNAMIC;
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
            rampedIdleRPM = getMeasuredShooterRPM();
            requestedIdleRPM = 0.0;
            requestShooterOff();
            return;
          }

          requestedIdleRPM = getRequestedIdleRPM(getSelectedIdleMode());
          double idleRPM = updateRampedIdleRPM(requestedIdleRPM);

          if (idleRPM <= ShooterConstants.idleMinCommandRPM.getAsDouble()) {
            requestShooterOff();
            return;
          }

          updateDefaultCoastState(idleRPM);

          if (defaultShouldCoast) {
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
        () -> requestShooterVelocity(
            ShooterModeState.IDLE, getRequestedIdleRPM(getSelectedIdleMode())));
  }

  /** Requests idle shooter speed once and finishes immediately. */
  public Command idleShooterInstant() {
    return runOnce(
        () -> {
          enableDefaultIdle();
          requestedIdleRPM = getRequestedIdleRPM(getSelectedIdleMode());
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
            <= Math.max(ShooterConstants.coastRPM.getAsDouble(), rampedIdleRPM)
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

    Logger.recordOutput(SHOOTER_TABLE_KEY + "Default/ShouldCoast", defaultShouldCoast);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Default/CoastEnterRPM", coastEnterRPM);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Default/CoastExitRPM", coastExitRPM);
  }

  /** Returns the unramped idle target for the selected idle mode. */
  private double getRequestedIdleRPM(ShooterIdleMode idleMode) {
    switch (idleMode) {
      case OFF:
        return 0.0;

      case PASSING:
        return ShooterConstants.passingIdleRPM.getAsDouble();

      case SHOOTING:
        return ShooterConstants.shootingIdleRPM.getAsDouble();

      case DYNAMIC:
        return getDynamicIdleRPM();

      case FIXED:
      default:
        return ShooterConstants.coastRPM.getAsDouble();
    }
  }

  /** Returns the dynamic idle target based on the current shooter lookup-table target. */
  private double getDynamicIdleRPM() {
    var params = ShooterCalculation.getInstance().getParameters();
    double lookupRPM =
        MathUtil.clamp(params.flywheelSpeed(), 0.0, ShooterConstants.MAX_FLYWHEEL_CALCULATED_RPM);
    double scaledRPM = lookupRPM * ShooterConstants.dynamicIdleScalar.getAsDouble();

    return MathUtil.clamp(
        scaledRPM,
        ShooterConstants.dynamicIdleMinRPM.getAsDouble(),
        ShooterConstants.dynamicIdleMaxRPM.getAsDouble());
  }

  /** Slews the idle target so idle spin-up is gentle and idle spin-down can coast. */
  private double updateRampedIdleRPM(double targetRPM) {
    double clampedTargetRPM = MathUtil.clamp(targetRPM, 0.0, ShooterConstants.MAX_FLYWHEEL_RPM);
    double maxDelta =
        (clampedTargetRPM > rampedIdleRPM
                ? ShooterConstants.idleRampUpRPMPerSec.getAsDouble()
                : ShooterConstants.idleRampDownRPMPerSec.getAsDouble())
            * Constants.loopPeriodSecs;

    rampedIdleRPM =
        MathUtil.clamp(clampedTargetRPM, rampedIdleRPM - maxDelta, rampedIdleRPM + maxDelta);

    return rampedIdleRPM;
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

    Logger.recordOutput(SHOOTER_TABLE_KEY + "ShooterMode", RobotState.getShooterMode().toString());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredVelocityRPM", outputs.desiredVelocityRPM);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "MeasuredVelocityRPM", getMeasuredShooterRPM());
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "VelocityErrorRPM",
        outputs.desiredVelocityRPM - getMeasuredShooterRPM());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredDutyCycle", outputs.desiredDutyCycle);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "UseMotionMagic", outputs.useMotionMagic);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/Mode", getSelectedIdleMode().toString());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/RequestedRPM", requestedIdleRPM);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Idle/RampedRPM", rampedIdleRPM);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "Tolerance/UnderRPM",
        ShooterConstants.flywheelUnderToleranceRPM.getAsDouble());
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "Tolerance/OverRPM",
        ShooterConstants.flywheelOverToleranceRPM.getAsDouble());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "RawShooterAtCurrentTarget", rawAtCurrentTarget);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "IsShooterReadyFiltered",
        !hasSpeedTargetChanged && rawAtCurrentTarget);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "BelowCoastRPM", isShooterBelowCoastRPM().getAsBoolean());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "HasSpeedTargetChanged", hasSpeedTargetChanged);

    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kP", outputs.kP);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kI", outputs.kI);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kD", outputs.kD);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kS", outputs.kS);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kV", outputs.kV);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/kA", outputs.kA);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/MMAcceleration", outputs.kMMAcceleration);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "Tuning/MMJerk", outputs.kMMJerk);
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