package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the shooter flywheel.
 *
 * <p>The shooter is commanded in RPM at the subsystem level. The IO layer converts RPM to the CTRE
 * rotations-per-second units required by the motor controllers.
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputsAutoLogged outputs = new ShooterIOOutputsAutoLogged();

  private boolean hasSpeedTargetChanged = true;
  private boolean defaultShouldCoast = true;

  /**
   * Creates a shooter subsystem using the provided hardware implementation.
   *
   * @param io shooter hardware abstraction
   */
  public Shooter(ShooterIO io) {
    this.io = io;

    if (Constants.Tuning.SHOOTER) {
      SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);
    }
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
          updateDefaultCoastState();

          if (defaultShouldCoast) {
            requestShooterOff();
          } else {
            requestShooterVelocity(ShooterModeState.IDLE, ShooterConstants.coastRPM.getAsDouble());
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
            ShooterModeState.IDLE, ShooterConstants.coastRPM.getAsDouble()));
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
            <= ShooterConstants.coastRPM.getAsDouble()
                + ShooterConstants.IDLE_COAST_EXIT_MARGIN_RPM;
  }

  /**
   * Updates whether the default command should coast or hold idle.
   *
   * <p>This small hysteresis state machine replaces the old timed {@code ConditionalCommand}.
   */
  private void updateDefaultCoastState() {
    double measuredRPM = getMeasuredShooterRPM();
    double idleRPM = ShooterConstants.coastRPM.getAsDouble();

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
    if (Math.abs(rpm - outputs.desiredVelocityRPM) > ShooterConstants.FLYWHEEL_TOLERANCE_RPM) {
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
    return Math.abs(getMeasuredShooterRPM() - outputs.desiredVelocityRPM)
        <= ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
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