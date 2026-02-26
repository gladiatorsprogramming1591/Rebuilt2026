package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(2), 0.004, ShooterConstants.SHOOTER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(2));

  private double shooterSpeed = 0.0;

  public void updateInputs(ShooterIOInputs inputs) {
    shooterSim.setInputVoltage(shooterSpeed);
    shooterSim.update(0.02);

    inputs.shooterSpeed = shooterSpeed;
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterSpeed = MathUtil.clamp(speed, -12.0, 12.0);
  }
}
