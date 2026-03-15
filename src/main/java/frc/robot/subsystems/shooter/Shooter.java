package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputsAutoLogged outputs = new ShooterIOOutputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.6);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.3);

  private static final LoggedTunableNumber shootRPM =
      new LoggedTunableNumber("Shooter/Shoot RPM", 2000);
  private static final LoggedTunableNumber coastRPM =
      new LoggedTunableNumber("Shooter/Coast RPM", 750);

  private double loopCounter = 0;
  private boolean hasSpeedTargetChanged = true;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    outputs.kP = kP.getAsDouble();
    outputs.kD = kD.getAsDouble();
    outputs.kV = kV.getAsDouble();

    boolean doApplyOutputs = true;
    switch (RobotState.getShooterMode()) {
      case DUTYCYCLE -> doApplyOutputs = false;
      default -> {
        // if(loopCounter++ % 25 == 0) System.out.println("Shooter mode : " +
        // RobotState.getShooterMode());
        // TODO***: Needs to only run once when coming out of DUTYCYCLE mode, not periodicly
        // (or removed if interrupted by KrakenIO applyOutputs anyway)
        io.runShooterDutyCycle(0);
      }
    }
    Logger.recordOutput("Shooter/Desired Velocity RPM", outputs.desiredVelocityRPM);
    SmartDashboard.putString("Shooter Mode", RobotState.getShooterMode().toString());
    SmartDashboard.putBoolean("Shooter DoApplyOutputs", doApplyOutputs);
    SmartDashboard.putBoolean("isShooterAtVelocity", isShooterAtVelocity().getAsBoolean());
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
          outputs.desiredVelocityRPM = coastRPM.getAsDouble();
        });
  }

  public Command runFixedSpeedCommand() {
    return run(
        () -> {
          if (RobotState.getShooterMode() != ShooterModeState.ON) {
            hasSpeedTargetChanged = true;
          }
          RobotState.setShooterMode(ShooterModeState.ON);
          outputs.desiredVelocityRPM = shootRPM.getAsDouble();
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
          double flywheelSpeedRPM =
              ShooterCalculation.getInstance().getParameters().flywheelSpeed();
          outputs.desiredVelocityRPM = MathUtil.clamp(flywheelSpeedRPM, 0, 3000);
        });
  }

  public Command runShooterDutyCycle(double dutyCycle) {
    SmartDashboard.putNumber("Shooter duty cycle", dutyCycle);
    return run(
        () -> {
          RobotState.setShooterMode(ShooterModeState.DUTYCYCLE);
          io.runShooterDutyCycle(dutyCycle);
        });
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
}
