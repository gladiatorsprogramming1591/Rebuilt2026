package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.INTAKE_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private DCMotorSim deploySim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.DEPLOY_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private double deployAppliedSpeed;
  private double deployAppliedTorqueCurrentFOC;
  private double intakeAppliedSpeed;

  public IntakeIOSim() {
    deployAppliedSpeed = 0.0;
    deployAppliedTorqueCurrentFOC = 0.0;
    intakeAppliedSpeed = 0.0;
  }

  @Override
  public void setDeploySpeed(double speed) {
    deployAppliedSpeed = speed;
  }

  @Override
  public void setDeployTorqueCurrentFOC(double current) {
    deployAppliedTorqueCurrentFOC = current;
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeAppliedSpeed = speed;
  }

  @Override
  public void stopDeployMotor() {
    deployAppliedSpeed = 0;
  }

  @Override
  public void stopIntakeMotor() {
    intakeAppliedSpeed = 0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deploySpeed = deployAppliedSpeed;
    inputs.deployTorqueCurrentFOC = deployAppliedTorqueCurrentFOC;
    inputs.intakeSpeed = intakeAppliedSpeed;
  }
}
