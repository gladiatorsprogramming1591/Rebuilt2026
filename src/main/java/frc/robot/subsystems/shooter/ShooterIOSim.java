package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
  public void runShooterTarget(double shooterVelocity) {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocity = shooterVelocity;
  }
}
