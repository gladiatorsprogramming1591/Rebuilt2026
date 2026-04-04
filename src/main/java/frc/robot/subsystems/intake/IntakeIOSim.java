package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

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
  private double deployStowDelay = 0.5;
  private final Timer timer = new Timer();
  private boolean isTimerRunning = false;

  // 0 = down, 1 = up, 2 = in between
  private LoggedTunableNumber deployState = new LoggedTunableNumber("Intake/Deploy State Sim", 0);

  public IntakeIOSim() {
    deployAppliedSpeed = 0.0;
    intakeAppliedSpeed = 0.0;
    deployAngle = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deploySpeed = deployAppliedSpeed;
    inputs.intakeLeftSpeed = intakeAppliedSpeed;
    inputs.intakeRightSpeed = intakeAppliedSpeed;
    inputs.deployPosition = deployAngle;
    // inputs.isDeployDown = deployState.getAsDouble() == 0;
    // inputs.isDeployUp = deployState.getAsDouble() == 1;
    inputs.isDeployDown = inputs.deployPosition == IntakeConstants.DOWN;
    inputs.isDeployUp = inputs.deployPosition == IntakeConstants.UP;
    if (inputs.isDeployUp) { // Re-zero when deploy is up
      inputs.encoderOffset = -deployAngle;
      inputs.deploySupplyCurrent = IntakeConstants.DEPLOY_CURRENT_STOP_THRESHOLD;
    } else if (inputs.isDeployDown) {
      inputs.deploySupplyCurrent = IntakeConstants.DEPLOY_CURRENT_STOP_THRESHOLD;
    } else {
      inputs.deploySupplyCurrent = 1.0; // Random non-zero current
    }
    inputs.deployPosition = deployAngle + inputs.encoderOffset;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    intakeAppliedSpeed = outputs.appliedIntakeSpeed;
    switch (RobotState.getDeployMode()) {
      case POSITION:
        deployAngle = outputs.desiredPosition;
        break;
      case SPEED:
        deployAppliedSpeed = outputs.appliedDeploySpeed;
        break;
      case OFF:
        deployAppliedSpeed = 0;
        break;
      default:
        System.out.println(
            "Intake Apply Outputs Empty Default" + RobotState.getDeployMode().toString());
        break;
    }
  }
}
