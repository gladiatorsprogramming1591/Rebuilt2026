package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerIOSim implements RollerIO {
  private double volts = 0.0;

  private final DCMotorSim intakeSim;
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double inertia = 1.0;
  private static final double gearRatio = 1.0;

  public RollerIOSim() {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, inertia, gearRatio), INTAKE_GEARBOX);
  }

  @Override
  public void setRollerSpeed(double volts) {
    this.volts = MathUtil.clamp(volts, 0, 12.0);
    SmartDashboard.putNumber("Roller Speed", this.volts);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerSpeed = this.volts;
  }
}
