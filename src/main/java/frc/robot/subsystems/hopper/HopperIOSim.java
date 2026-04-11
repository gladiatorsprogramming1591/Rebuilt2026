package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
  private double speed = 0.0;

  private final DCMotorSim intakeSim;
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double inertia = 1.0;
  private static final double gearRatio = 1.0;

  public HopperIOSim() {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, inertia, gearRatio), INTAKE_GEARBOX);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {}

  @Override
  public void applyOutputs(HopperIOOutputs outputs) {
    if (!outputs.usingLowerCurrent && outputs.useBeltWhileIntakeCurrent) {
      outputs.usingLowerCurrent = true;
    } else if (outputs.usingLowerCurrent && !outputs.useBeltWhileIntakeCurrent) {
      outputs.usingLowerCurrent = false;
    }
  }
}
