package frc.robot.subsystems.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerSpeed = speed;
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    speed = outputs.desiredKickerSpeed;
  }
}
