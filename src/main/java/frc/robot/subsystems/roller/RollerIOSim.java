package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public void setSpeed(double speed) {
    this.speed = speed;
    SmartDashboard.putNumber("Intake Speed", speed);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.speed = this.speed;
  }
}
