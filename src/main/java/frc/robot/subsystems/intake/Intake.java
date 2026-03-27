package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakeModeState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();

  private final LoggedTunableNumber deploySpeed =
      new LoggedTunableNumber("Intake/DeploySpeed", IntakeConstants.DEPLOY_SPEED);
  private final LoggedTunableNumber intakeSpeed =
      new LoggedTunableNumber("Intake/IntakeSpeed", IntakeConstants.INTAKE_MOTOR_SPEED);
  private final LoggedTunableNumber stowSpeed =
      new LoggedTunableNumber("Intake/StowSpeed", IntakeConstants.STOW_SPEED);
  private final LoggedTunableNumber deployCurrent =
      new LoggedTunableNumber("Intake/DeployCurrent", IntakeConstants.DEPLOY_TORQUE_CURRENT);
  private final LoggedTunableNumber intakeDelaySeconds =
      new LoggedTunableNumber("Intake/IntakeDelaySeconds", IntakeConstants.INTAKE_DELAY_SECONDS);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/kP", IntakeConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/kI", IntakeConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  private static final LoggedTunableNumber kFF =
      new LoggedTunableNumber("Intake/kFF", IntakeConstants.kFF);

  private static final double intakeMaxAngle =
      Units.degreesToRadians(0); // TODO set the max and min angle
  private static final double intakeMinAngle =
      Units.degreesToRadians(0); // TODO set the max and min angle

  @AutoLogOutput private double goalAngle = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void runPosition(double positionRads) {
    goalAngle = positionRads;
    outputs.desiredPosition = MathUtil.clamp(goalAngle, intakeMinAngle, intakeMaxAngle);
  }

  // TODO: Change from runEnd to let applyOutputs check for down and stop motor. Probably need new
  // states.
  public Command deploy() {
    return runEnd(
        () -> {
          outputs.desiredPosition = IntakeConstants.UP;
          RobotState.setIntakeMode(RobotState.IntakeModeState.POSITION);
        },
        () -> {
          outputs.appliedDeploySpeed = 0;
          RobotState.setIntakeMode(RobotState.IntakeModeState.OFF);
        });
  }

  // TODO: Change from runEnd to let applyOutputs check for up and stop motor. Probably need new
  // states.
  public Command stow() {
    return runEnd(
        () -> {
          outputs.desiredPosition = IntakeConstants.DOWN;
          RobotState.setIntakeMode(RobotState.IntakeModeState.POSITION);
        },
        () -> {
          outputs.appliedDeploySpeed = 0;
          RobotState.setIntakeMode(RobotState.IntakeModeState.OFF);
        });
  }

  public Command deployWithSpeed() {
    return runEnd(
      () -> {
          System.out.println("deployWithSpeed: " + deploySpeed.getAsDouble());
          outputs.appliedDeploySpeed = deploySpeed.getAsDouble();
          RobotState.setIntakeMode(RobotState.IntakeModeState.SPEED);
        },
        () -> {
          System.out.println("deployWithSpeed: " + 0);
          outputs.appliedDeploySpeed = 0;
          RobotState.setIntakeMode(RobotState.IntakeModeState.OFF);
        });
  }

  // TODO: Need to implement
  public Command deployAndIntake() {
    return new InstantCommand();
  }

  // TODO: Need to implement
  public Command stopIntake() {
    return run(
        () -> {
          outputs.appliedIntakeSpeed = 0;
          outputs.appliedDeploySpeed = 0;
          RobotState.setIntakeMode(IntakeModeState.OFF);
        });
  }

  /** Returns a command that sets intake speed to 0. Doesn't impact deploy. */
  public Command stopIntakeInstant() {
    return new InstantCommand(
        () -> {
          outputs.appliedIntakeSpeed = 0;
        });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    outputs.kP = kP.getAsDouble();
    outputs.kI = kI.getAsDouble();
    outputs.kD = kD.getAsDouble();
    outputs.kFF = kFF.getAsDouble();

    Logger.recordOutput("Intake/Mode", RobotState.getIntakeMode().toString());
    Logger.recordOutput("Intake/Applied Intake Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput("Intake/Applied Output Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput("Intake/Applied Deploy Current", outputs.appliedDeployCurrent);
    Logger.recordOutput("Intake/kP", outputs.kP);
    Logger.recordOutput("Intake/kI", outputs.kI);
    Logger.recordOutput("Intake/kD", outputs.kD);
    Logger.recordOutput("Intake/kFF", outputs.kFF);
    Logger.recordOutput("Intake/desiredPosition", outputs.desiredPosition);
    io.applyOutputs(outputs);
  }
}
