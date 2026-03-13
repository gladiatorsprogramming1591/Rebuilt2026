package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(2), 0.004, ShooterConstants.SHOOTER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(2));

  private double shooterVelocity = 0.0;

  @Override
  public void runShooter(double shooterVelocity) {
    this.shooterVelocity = shooterVelocity;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.LF_RPS = inputs.LL_RPS = inputs.RF_RPS = inputs.RL_RPS = shooterVelocity;
    inputs.shooterAtVelocity = true;
  }

  @Override
  public BooleanSupplier shooterAtVelocity() {
    return () -> true;
  }

  @Override
  public void setShooterMotorRPM(double rps) {
    shooterVelocity = rps;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    double lastCommandedVelocity = outputs.desiredVelocityRPM / 60;

    // Logger.recordOutput("Shooter/lastCommandedVelocity", lastCommandedVelocity);
    SmartDashboard.putNumber("lastCommandedVelocity", lastCommandedVelocity);
  }
}
