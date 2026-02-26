package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerIOSim implements KickerIO {
  private double speed = 0.0;

  public KickerIOSim() {
    // Simulate the motor with a DCMotorSim
    DCMotorSim kickerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void setKickerSpeed(double speed) {
    this.speed = MathUtil.clamp(speed, 0, 1.0);
    SmartDashboard.putNumber("Kicker Speed", this.speed);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerSpeed = this.speed;
  }
}
