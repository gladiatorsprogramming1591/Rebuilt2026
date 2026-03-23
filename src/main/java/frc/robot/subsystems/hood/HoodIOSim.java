package frc.robot.subsystems.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
  private double speed = 0.0;
  private double angle = 0.0;

  public HoodIOSim() {
    // Simulate the motor with a DCMotorSim
    DCMotorSim hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX44Foc(1));
  }

  @Override
  public boolean isHoodAtTrueZero() {
    return true;
  }

  // @Override
  // public void setHoodSpeed(double speed) {
  //   this.speed = MathUtil.clamp(speed, 0, 1.0);
  //   SmartDashboard.putNumber("Hood Speed", this.speed);
  // }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodSpeed = speed;
    inputs.hoodAngle = angle;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodMode.POSITION) {
      angle = outputs.desiredHoodAngle;
    } else if (outputs.mode == HoodMode.SPEED) {
      speed = outputs.desiredHoodSpeed;
      angle += speed * 0.02 / HoodConstants.HOOD_MOTOR_REDUCTION;
    }
  }
}
