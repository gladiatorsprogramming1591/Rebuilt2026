package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
   * Runs the shooter at the configured idle/coast RPM.
   *
   * @return command that idles the shooter while scheduled
   */
  public Command runIdleCommand() {
    return run(
        () -> {
          setShooterMode(ShooterModeState.IDLE);
          setDesiredVelocityRPM(ShooterConstants.coastRPM.getAsDouble());
        });
  }

  /**
   * Runs the shooter at the fixed tunable shooting RPM.
   *
   * @return command that runs the shooter at fixed RPM while scheduled
   */
  public Command runFixedSpeedCommand() {
    return run(
        () -> {
          setShooterMode(ShooterModeState.ON);
          setDesiredVelocityRPM(ShooterConstants.shootFixedRPM.getAsDouble());
        });
  }

  /**
   * Runs the shooter at the calculated target RPM from {@link ShooterCalculation}.
   *
   * @return command that continuously updates the shooter target while scheduled
   */
  public Command runShooterTarget() {
    return run(
        () -> {
          setShooterMode(ShooterModeState.ON);

          var params = ShooterCalculation.getInstance().getParameters();
          double flywheelRPM =
              MathUtil.clamp(
                  params.flywheelSpeed(), 0.0, ShooterConstants.MAX_FLYWHEEL_CALCULATED_RPM);

          setDesiredVelocityRPM(flywheelRPM);

          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target/Passing", params.passing());
          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target/FlywheelRPM", params.flywheelSpeed());
          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target/CommandedFlywheelRPM", outputs.desiredVelocityRPM);
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
          outputs.desiredDutyCycle = MathUtil.clamp(dutyCycle, -1.0, 1.0);
        });
  }

  /**
   * Stops the shooter motor output and lets the flywheel coast.
   *
   * @return command that keeps the shooter off while scheduled
   */
  public Command stopAndCoastShooter() {
    return run(
        () -> {
          setShooterMode(ShooterModeState.OFF);
          setDesiredVelocityRPM(0.0);
          outputs.desiredDutyCycle = 0.0;
        });
  }

  /**
   * Chooses the default shooter behavior based on current flywheel speed.
   *
   * <p>If the shooter is below coast speed, it idles. Otherwise, it turns off and coasts down.
   *
   * @return conditional default command for the shooter
   */
  public ConditionalCommand coastShooterDefaultCommand() {
    ConditionalCommand command =
        new ConditionalCommand(runIdleCommand(), stopAndCoastShooter(), isShooterBelowCoastRPM());
    command.addRequirements(this);
    return command;
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
   * Returns whether the shooter is below coast RPM.
   *
   * @return true when the right leader is below coast RPM plus tolerance
   */
  public BooleanSupplier isShooterBelowCoastRPM() {
    return () -> {
      if (hasSpeedTargetChanged) {
        hasSpeedTargetChanged = false;
        return false;
      }

      return io.rightShooterBelowCoastRPM().getAsBoolean();
    };
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

  private boolean rawShooterAtCurrentTarget() {
    return io.rightShooterAtVelocityRPM(() -> outputs.desiredVelocityRPM).getAsBoolean();
  }

  /** Logs requested shooter state and readiness values. */
  private void logShooterState() {
    boolean rawAtCurrentTarget = rawShooterAtCurrentTarget();

    Logger.recordOutput(SHOOTER_TABLE_KEY + "ShooterMode", RobotState.getShooterMode().toString());
    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredVelocityRPM", outputs.desiredVelocityRPM);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredDutyCycle", outputs.desiredDutyCycle);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "UseMotionMagic", outputs.useMotionMagic);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "RawShooterAtCurrentTarget", rawAtCurrentTarget);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "IsShooterReadyFiltered",
        !hasSpeedTargetChanged && rawAtCurrentTarget);
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + "BelowCoastRPM", io.rightShooterBelowCoastRPM().getAsBoolean());
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