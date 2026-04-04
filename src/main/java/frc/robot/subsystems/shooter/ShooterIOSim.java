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

  private double shooterRPS = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.LF_RPS = inputs.LL_RPS = inputs.RF_RPS = inputs.RL_RPS = shooterRPS;
    inputs.shooterAtVelocity = true;
  }

  @Override
  public BooleanSupplier rightShooterAtVelocity() {
    return () -> true;
  }

  @Override
  public void setShooterMotorRPM(double rps) {
    shooterRPS = rps;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    double lastCommandedVelocity = outputs.desiredVelocityRPM / 60;

    // Logger.recordOutput(ShooterConstants.tableKey + "lastCommandedVelocity", lastCommandedVelocity);
    SmartDashboard.putNumber("lastCommandedVelocityRPS", lastCommandedVelocity);
  }
}
