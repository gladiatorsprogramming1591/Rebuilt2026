package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotState;

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
  private double intakeAppliedSpeed;
  private double deployAngle;

  public IntakeIOSim() {
    deployAppliedSpeed = 0.0;
    intakeAppliedSpeed = 0.0;
    deployAngle = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deploySpeed = deployAppliedSpeed;
    inputs.intakeLeftSpeed = intakeAppliedSpeed;
    inputs.deployPosition = deployAngle;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    intakeAppliedSpeed = outputs.appliedIntakeSpeed;
    switch (RobotState.getIntakeMode()) {
      case POSITION:
        deployAngle = outputs.desiredPosition;
        break;
      case SPEED:
        deployAppliedSpeed = outputs.appliedDeploySpeed;
        break;
      default:
        System.out.println("Intake Apply Outputs Empty Default");
        break;
    }
  }
}
