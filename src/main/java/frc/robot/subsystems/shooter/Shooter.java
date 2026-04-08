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
    if (Constants.tuningMode) {SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);}
    SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + "below coast RPM", true);
    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Duty-cycle", 0.0);
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

    if (Constants.tuningMode) {io.tuneMotorConfigs(outputs);}

    boolean doApplyOutputs = true;
    if (RobotState.getShooterMode() == ShooterModeState.DUTYCYCLE) {
      doApplyOutputs = false;
    }
    if (ShooterConstants.isLowCeiling && RobotState.getShooterMode() == ShooterModeState.ON) {
      outputs.desiredVelocityRPM =
          MathUtil.clamp(
              outputs.desiredVelocityRPM * ShooterConstants.FLYWHEEL_LOW_CEILING_SCALER,
              -Double.MAX_VALUE,
              ShooterConstants.MAX_FLYWHEEL_LOW_CEILING_RPM);
    }

    Logger.recordOutput(ShooterConstants.SHOOTER_TABLE_KEY + "Desired Velocity RPM", outputs.desiredVelocityRPM);
    SmartDashboard.putString(ShooterConstants.SHOOTER_TABLE_KEY + "Shooter Mode", RobotState.getShooterMode().toString());
    SmartDashboard.putBoolean(ShooterConstants.SHOOTER_TABLE_KEY + "DoApplyOutputs", doApplyOutputs);
    SmartDashboard.putBoolean(ShooterConstants.SHOOTER_TABLE_KEY + "isShooterAtVelocity", isShooterAtVelocity().getAsBoolean());
    SmartDashboard.putBoolean(ShooterConstants.SHOOTER_TABLE_KEY + "below coast RPM", isShooterBelowCoastRPM().getAsBoolean());
    SmartDashboard.putBoolean(ShooterConstants.SHOOTER_TABLE_KEY + "hasSpeedTargetChanged", hasSpeedTargetChanged);

    if (doApplyOutputs) {
      io.applyOutputs(outputs);
    }
  }

  public Command runIdleCommand() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.IDLE) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.IDLE);
          outputs.desiredVelocityRPM = ShooterConstants.coastRPM.getAsDouble();
        });
  }

  public Command runFixedSpeedCommand() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.ON) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.ON);
          outputs.desiredVelocityRPM = ShooterConstants.shootFixedRPM.getAsDouble();
        });
  }

  public Command runShooterTarget() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.IDLE) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.ON);
          // outputs.velocityRPM = shootRPM.getAsDouble();
          double flywheelRPM = ShooterCalculation.getInstance().getParameters().flywheelSpeed();
          outputs.desiredVelocityRPM =
              MathUtil.clamp(flywheelRPM, 0.0, ShooterConstants.MAX_FLYWHEEL_CALCULATED_RPM);
        });
  }

  public Command runShooterDutyCycle(double dutyCycle) {
    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Duty-cycle", dutyCycle);
    return run(
        () -> {
          RobotState.setShooterMode(ShooterModeState.DUTYCYCLE);
          io.runShooterDutyCycle(dutyCycle);
        });
  }

  /**
   * Set duty-cycle to 0 to allow shooter to coast down to zero.
   * @return
   */
  public Command stopAndCoastShooter()
  {
    return runShooterDutyCycle(0);
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

  public BooleanSupplier isShooterAtVelocity() {
    return (() -> {
      if (hasSpeedTargetChanged) {
        hasSpeedTargetChanged = false;
        return false;
      }
      return inputs.shooterAtVelocity;
    });
  }
  
  public BooleanSupplier isShooterBelowCoastRPM() {
    if (hasSpeedTargetChanged) {
      hasSpeedTargetChanged = false;
      return () -> false;
    }
    return io.rightShooterBelowCoastRPM();
  };
}
