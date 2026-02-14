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

  private double intakeAppliedVolts = 0.0;
  private double deployAppliedVolts = 0.0;

  public IntakeIOSim() {
    intakeAppliedVolts = 0.5;
    deployAppliedVolts = 0.5;
  }

  @Override
  public void setDeployMotorVoltage(double volts) {
    deployAppliedVolts = volts;
  }

  @Override
  public void setIntakeMotorVoltage(double volts) {
    intakeAppliedVolts = volts;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.deployAppliedVolts = deployAppliedVolts;
  }
}
