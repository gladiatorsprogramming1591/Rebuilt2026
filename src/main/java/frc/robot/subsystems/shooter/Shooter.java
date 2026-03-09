package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterModeState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO.ShooterIOOutputs outputs = new ShooterIO.ShooterIOOutputs();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.6);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.3);

  private static final LoggedTunableNumber shootRPM =
      new LoggedTunableNumber("Shooter/Shoot RPM", 2000);
  private static final LoggedTunableNumber coastRPM =
      new LoggedTunableNumber("Shooter/Coast RPM", 750);

  private double loopCounter = 0;

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
        // if(loopCounter++ % 25 == 0) System.out.println("Shooter mode : " + RobotState.getShooterMode());
        io.runShooterVelocity(0);
      }
    }
    SmartDashboard.putString("Shooter Mode", RobotState.getShooterMode().toString());
    SmartDashboard.putBoolean("Shooter DoApplyOutputs", doApplyOutputs);
    if (doApplyOutputs) io.applyOutputs(outputs);
  }

  public Command runIdleCommand() {
    return run(
        () -> {
          RobotState.setShooterMode(ShooterModeState.IDLE);
          outputs.desiredVelocityRPM = coastRPM.getAsDouble();
        });
  }

  public Command runShooterTarget() {
    return run(
        () -> {
          RobotState.setShooterMode(ShooterModeState.ON);
          // outputs.velocityRPM = shootRPM.getAsDouble();
          double flywheelSpeedRadPerSec =
              ShooterCalculation.getInstance().getParameters().flywheelSpeed();
          outputs.desiredVelocityRPM =
              flywheelSpeedRadPerSec * (2 * Math.PI) * 60; // Convert to Rotations per minute
        });
  }

  public Command runShooterVelocity(double velocity) {
    SmartDashboard.putNumber("Shooter Vel", velocity);
    return run(
        () -> {
          RobotState.setShooterMode(ShooterModeState.DUTYCYCLE);
          io.runShooterVelocity(velocity);
        });
  }

  // public boolean isShooterAtVelocity(double velocity) {
  //   return MathUtil.isNear(
  //     velocity,
  //     , velocity)
  // }

}
