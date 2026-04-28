package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputsAutoLogged outputs = new ShooterIOOutputsAutoLogged();

  private boolean hasSpeedTargetChanged = true;

  public Shooter(ShooterIO io) {
    this.io = io;
    if (Constants.Tuning.SHOOTER) {
      SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);
    }
    SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + "below coast RPM", true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    outputs.useMotionMagic = ShooterConstants.useMotionMagic.getAsBoolean();
    outputs.kP = ShooterConstants.kP.getAsDouble();
    outputs.kI = ShooterConstants.kI.getAsDouble();
    outputs.kD = ShooterConstants.kD.getAsDouble();
    outputs.kS = ShooterConstants.kS.getAsDouble();
    outputs.kV = ShooterConstants.kV.getAsDouble();
    outputs.kA = ShooterConstants.kA.getAsDouble();
    outputs.kMMAcceleration = ShooterConstants.kMMAcceleration.getAsDouble();
    outputs.kMMJerk = ShooterConstants.kMMJerk.getAsDouble();

    if (Constants.Tuning.SHOOTER) {
      io.tuneMotorConfigs(outputs);
    }

    if (ShooterConstants.isLowCeiling && RobotState.getShooterMode() == ShooterModeState.ON) {
      outputs.desiredVelocityRPM =
          MathUtil.clamp(
              outputs.desiredVelocityRPM * ShooterConstants.FLYWHEEL_LOW_CEILING_SCALER,
              -Double.MAX_VALUE,
              ShooterConstants.MAX_FLYWHEEL_LOW_CEILING_RPM);
    }

    Logger.recordOutput(ShooterConstants.SHOOTER_TABLE_KEY + "Desired Velocity RPM", outputs.desiredVelocityRPM);
    Logger.recordOutput(ShooterConstants.SHOOTER_TABLE_KEY + "Desired Duty-Cycle", outputs.desiredDutyCycle);
    SmartDashboard.putString(ShooterConstants.SHOOTER_TABLE_KEY + "Shooter Mode", RobotState.getShooterMode().toString());
    boolean rawAtCurrentTarget = rawShooterAtCurrentTarget();

    SmartDashboard.putBoolean(
        ShooterConstants.SHOOTER_TABLE_KEY + "rawShooterAtCurrentTarget",
        rawAtCurrentTarget);
    SmartDashboard.putBoolean(
        ShooterConstants.SHOOTER_TABLE_KEY + "isShooterReadyFiltered",
        !hasSpeedTargetChanged && rawAtCurrentTarget);
    SmartDashboard.putBoolean(
        ShooterConstants.SHOOTER_TABLE_KEY + "below coast RPM",
        io.rightShooterBelowCoastRPM().getAsBoolean());
    SmartDashboard.putBoolean(ShooterConstants.SHOOTER_TABLE_KEY + "hasSpeedTargetChanged", hasSpeedTargetChanged);

    io.applyOutputs(outputs);
  }

  private void setDesiredVelocityRPM(double rpm) {
  if (Math.abs(rpm - outputs.desiredVelocityRPM)
      > ShooterConstants.FLYWHEEL_TOLERANCE_RPS * 60.0) {
    hasSpeedTargetChanged = true;
  }

  outputs.desiredVelocityRPM = rpm;
}

private boolean rawShooterAtCurrentTarget() {
  return io.rightShooterAtVelocity(() -> outputs.desiredVelocityRPM / 60.0).getAsBoolean();
}

  public Command runIdleCommand() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.IDLE) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.IDLE);
          setDesiredVelocityRPM(ShooterConstants.coastRPM.getAsDouble());
        });
  }

  public Command runFixedSpeedCommand() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.ON) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.ON);
          setDesiredVelocityRPM(ShooterConstants.shootFixedRPM.getAsDouble());
        });
  }

  public Command stopAndCoastShooter() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.OFF) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.OFF);
          setDesiredVelocityRPM(0.0);
          outputs.desiredDutyCycle = 0.0;
        });
  }

  public Command runShooterTarget() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.ON) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.ON);
          var params = ShooterCalculation.getInstance().getParameters();
          double flywheelRPM =
              MathUtil.clamp(
                  params.flywheelSpeed(), 0.0, ShooterConstants.MAX_FLYWHEEL_CALCULATED_RPM);

          setDesiredVelocityRPM(flywheelRPM);

          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target Passing", params.passing());
          Logger.recordOutput(SHOOTER_TABLE_KEY + "Target Flywheel RPM", params.flywheelSpeed());
          Logger.recordOutput(SHOOTER_TABLE_KEY + "Commanded Flywheel RPM", outputs.desiredVelocityRPM);
          SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + "Target Passing", params.passing());
          SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Target Flywheel RPM", params.flywheelSpeed());
          SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Commanded Flywheel RPM", outputs.desiredVelocityRPM);
        });
  }

  public Command runShooterDutyCycle(double dutyCycle) {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.DUTYCYCLE) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.DUTYCYCLE);
          outputs.desiredDutyCycle = dutyCycle;
        });
  }

  public ConditionalCommand coastShooterDefaultCommand()
  {
    // return runEnd(
    //   () -> stopAndCoastShooter().until(isShooterBelowCoastRPM()), 
    //   () -> runIdleCommand());
    ConditionalCommand conditionalCommand = new ConditionalCommand(
      runIdleCommand(),
      stopAndCoastShooter(),
      isShooterBelowCoastRPM());
      conditionalCommand.addRequirements(this);
    return conditionalCommand;
  }

  public boolean getRawShooterAtVelocityForDebug() {
  return inputs.shooterAtVelocity;
}

public boolean getHasSpeedTargetChangedForDebug() {
  return hasSpeedTargetChanged;
}

public double getDesiredVelocityRPMForDebug() {
  return outputs.desiredVelocityRPM;
}

  public BooleanSupplier isShooterAtVelocity() {
    return () -> {
      if (hasSpeedTargetChanged) {
        hasSpeedTargetChanged = false;
        return false;
      }

      return rawShooterAtCurrentTarget();
    };
  }
  
  public BooleanSupplier isShooterBelowCoastRPM() {
    if (hasSpeedTargetChanged) {
      hasSpeedTargetChanged = false;
      return () -> false;
    }
    return io.rightShooterBelowCoastRPM();
  };
}
