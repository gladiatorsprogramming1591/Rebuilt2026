package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodIOSim implements HoodIO {
  private double speed = 0.0;

  public HoodIOSim() {
    // Simulate the motor with a DCMotorSim
    DCMotorSim hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public void setHoodSpeed(double speed) {
    this.speed = MathUtil.clamp(speed, 0, 1.0);
    SmartDashboard.putNumber("Hood Speed", this.speed);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodSpeed = this.speed;
  }
}
