package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Simple simulated shooter IO implementation.
 *
 * <p>This is not intended to be a full physics simulation. It exists so the shooter subsystem can run
 * in sim/replay without real hardware while following the same input/output structure as the real IO
 * implementation.
 */
public class ShooterIOSim implements ShooterIO {
  private static final double CLOSED_LOOP_ACCEL_RPM_PER_SECOND = 12000.0;
  private static final double COAST_DECEL_RPM_PER_SECOND = 2500.0;

  private double velocityRPM = 0.0;
  private double dutyCycle = 0.0;
  private double desiredVelocityRPM = 0.0;

  /** Creates a simple simulated shooter with all motors stopped. */
  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rightLeaderConnected = true;
    inputs.rightFollowerConnected = true;
    inputs.leftLeaderConnected = true;
    inputs.leftFollowerConnected = true;

    inputs.rightLeaderVelocityRPM = velocityRPM;
    inputs.rightFollowerVelocityRPM = velocityRPM;
    inputs.leftLeaderVelocityRPM = velocityRPM;
    inputs.leftFollowerVelocityRPM = velocityRPM;

    inputs.rightLeaderAppliedVolts = dutyCycle * 12.0;
    inputs.rightFollowerAppliedVolts = dutyCycle * 12.0;
    inputs.leftLeaderAppliedVolts = dutyCycle * 12.0;
    inputs.leftFollowerAppliedVolts = dutyCycle * 12.0;

    inputs.rightLeaderSupplyCurrent =
        Math.abs(dutyCycle) * ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    inputs.rightFollowerSupplyCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftLeaderSupplyCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftFollowerSupplyCurrent = inputs.rightLeaderSupplyCurrent;

    inputs.rightLeaderStatorCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.rightFollowerStatorCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftLeaderStatorCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftFollowerStatorCurrent = inputs.rightLeaderSupplyCurrent;

    inputs.rightLeaderTorqueCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.rightFollowerTorqueCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftLeaderTorqueCurrent = inputs.rightLeaderSupplyCurrent;
    inputs.leftFollowerTorqueCurrent = inputs.rightLeaderSupplyCurrent;

    inputs.rightLeaderTemperature = 25.0;
    inputs.rightFollowerTemperature = 25.0;
    inputs.leftLeaderTemperature = 25.0;
    inputs.leftFollowerTemperature = 25.0;

    inputs.shooterAtVelocity = rightShooterAtVelocity().getAsBoolean();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    desiredVelocityRPM = outputs.desiredVelocityRPM;

    switch (RobotState.getShooterMode()) {
      case ON:
      case IDLE:
        updateClosedLoopVelocity(desiredVelocityRPM);
        break;

      case DUTYCYCLE:
        dutyCycle = MathUtil.clamp(outputs.desiredDutyCycle, -1.0, 1.0);
        velocityRPM = dutyCycle * ShooterConstants.MAX_FLYWHEEL_RPM;
        break;

      case OFF:
        updateCoastVelocity();
        break;

      default:
        dutyCycle = 0.0;
        updateCoastVelocity();
        break;
    }
  }

  @Override
  public BooleanSupplier rightShooterAtVelocity() {
    return rightShooterAtVelocityRPM(() -> desiredVelocityRPM);
  }

  @Override
  public BooleanSupplier rightShooterAtVelocityRPM(DoubleSupplier targetRPM) {
    return () ->
        Math.abs(velocityRPM - targetRPM.getAsDouble())
            <= ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
  }

  @Override
  public BooleanSupplier rightShooterBelowCoastRPM() {
    return () ->
        velocityRPM
            <= ShooterConstants.coastRPM.getAsDouble()
                + ShooterConstants.IDLE_COAST_EXIT_MARGIN_RPM;
  }

  /**
   * Moves the simulated flywheel toward the closed-loop target at a limited acceleration rate.
   *
   * @param targetRPM requested closed-loop target in RPM
   */
  private void updateClosedLoopVelocity(double targetRPM) {
    double maxDelta = CLOSED_LOOP_ACCEL_RPM_PER_SECOND * Constants.loopPeriodSecs;
    velocityRPM = MathUtil.clamp(targetRPM, velocityRPM - maxDelta, velocityRPM + maxDelta);
    dutyCycle = MathUtil.clamp(velocityRPM / ShooterConstants.MAX_FLYWHEEL_RPM, -1.0, 1.0);
  }

  /** Lets the simulated flywheel coast down instead of instantly stopping. */
  private void updateCoastVelocity() {
    double maxDelta = COAST_DECEL_RPM_PER_SECOND * Constants.loopPeriodSecs;

    if (velocityRPM > 0.0) {
      velocityRPM = Math.max(0.0, velocityRPM - maxDelta);
    } else if (velocityRPM < 0.0) {
      velocityRPM = Math.min(0.0, velocityRPM + maxDelta);
    }

    dutyCycle = 0.0;
  }
}