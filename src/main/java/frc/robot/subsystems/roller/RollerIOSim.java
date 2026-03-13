package frc.robot.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private double speed = 0.0;

  private final DCMotorSim intakeSim;
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double inertia = 1.0;
  private static final double gearRatio = 1.0;

  public RollerIOSim() {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, inertia, gearRatio), INTAKE_GEARBOX);
  }

  // @Override
  // public void setTopRollerSpeed(double speed) {
  //   this.speed = MathUtil.clamp(speed, 0, 1.0);
  // }

  // @Override
  // public void setBottomRollerSpeed(double speed) {
  //   this.speed = speed;
  // }

  @Override
  public void updateInputs(RollerIOInputs inputs) {}

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {}
}
