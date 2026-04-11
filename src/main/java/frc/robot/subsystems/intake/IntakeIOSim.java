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
              DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.ROLLER_MOTOR_REDUCTION),
          DCMotor.getKrakenX44Foc(1));

  private DCMotorSim deploySim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44Foc(1), 0.004, IntakeConstants.SLAPDOWN_MOTOR_REDUCTION),
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
    inputs.slapdownSpeed = deployAppliedSpeed;
    inputs.rollerLeftSpeed = intakeAppliedSpeed;
    inputs.rollerRightSpeed = intakeAppliedSpeed;
    inputs.position = deployAngle;
    // inputs.isDeployDown = deployState.getAsDouble() == 0;
    // inputs.isDeployUp = deployState.getAsDouble() == 1;
    inputs.isSlapdownDown = inputs.position == IntakeConstants.DOWN;
    inputs.isSlapdownUp = inputs.position == IntakeConstants.UP;
    if (inputs.isSlapdownUp) { // Re-zero when deploy is up
      inputs.encoderOffset = -deployAngle;
      inputs.slapdownSupplyCurrent = IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD;
    } else if (inputs.isSlapdownDown) {
      inputs.slapdownSupplyCurrent = IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD;
    } else {
      inputs.slapdownSupplyCurrent = 1.0; // Random non-zero current
    }
    inputs.position = deployAngle + inputs.encoderOffset;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    intakeAppliedSpeed = outputs.appliedRollerSpeed;
    switch (RobotState.getSlapdownMode()) {
      case DEPLOY_POSITION: case STOW_POSITION: case BUMP_POSITION:
        deployAngle = outputs.desiredPosition;
        break;
      case SPEED:
        deployAppliedSpeed = outputs.appliedSlapdownSpeed;
        break;
      case OFF:
        deployAppliedSpeed = 0;
        break;
      default:
        System.out.println(
            "Intake Apply Outputs Empty Default" + RobotState.getSlapdownMode().toString());
        break;
    }
  }
}
