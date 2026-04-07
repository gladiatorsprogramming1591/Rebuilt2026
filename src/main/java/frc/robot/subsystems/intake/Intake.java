package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.DeployModeState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();

  private final LoggedTunableNumber deploySpeed =
      new LoggedTunableNumber("Intake/DeploySpeed", IntakeConstants.DEPLOY_SPEED);
  // private final LoggedTunableNumber intakeSpeed =
  //     new LoggedTunableNumber("Intake/IntakeSpeed", IntakeConstants.INTAKE_MOTOR_SPEED);
  // private final LoggedTunableNumber stowSpeed =
  //     new LoggedTunableNumber("Intake/StowSpeed", IntakeConstants.STOW_SPEED);
  // private final LoggedTunableNumber deployCurrent =
  //     new LoggedTunableNumber("Intake/DeployCurrent", IntakeConstants.DEPLOY_TORQUE_CURRENT);
  // private final LoggedTunableNumber intakeDelaySeconds =
  //     new LoggedTunableNumber("Intake/IntakeDelaySeconds", IntakeConstants.INTAKE_DELAY_SECONDS);

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/kP", IntakeConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/kI", IntakeConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  // private static final LoggedTunableNumber kS =
  //     new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  // private static final LoggedTunableNumber kV =
  //     new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  // private static final LoggedTunableNumber kA =
  //     new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  private static final LoggedTunableNumber kFF =
      new LoggedTunableNumber("Intake/kFF", IntakeConstants.kFF);

  private static final double intakeMaxAngle =
      Units.degreesToRadians(0); // TODO set the max and min angle
  private static final double intakeMinAngle =
      Units.degreesToRadians(0); // TODO set the max and min angle

  private int stoppedLoopCount = 0;
  private static final int STOPPED_LOOP_COUNT_NEEDED = 2;
  private boolean stopDeployOnCurrentSpike = false;
  private boolean stopMotorWhenNotMoving = false;
  private boolean isDeployStopped = true;

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
              isDeployStopped = false;
              outputs.desiredPosition = IntakeConstants.DOWN;
              RobotState.setDeployMode(RobotState.DeployModeState.POSITION);
              stopDeployOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedDeploySpeed = 0;
              RobotState.setDeployMode(RobotState.DeployModeState.OFF);
            })
        .until(() -> isDeployStopped);
  }

  // TODO: Change from runEnd to let applyOutputs check for up and stop motor. Probably need new
  // states.
  public Command stow() {
    return runEnd(
            () -> {
              isDeployStopped = false;
              outputs.desiredPosition = IntakeConstants.UP;
              RobotState.setDeployMode(RobotState.DeployModeState.POSITION);
              stopDeployOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedDeploySpeed = 0;
              RobotState.setDeployMode(RobotState.DeployModeState.OFF);
            })
        .until(() -> isDeployStopped);
  }

  public Command deployWithSpeed() {
    return runEnd(
        () -> {
          isDeployStopped = false;
          System.out.println("deployWithSpeed: " + deploySpeed.getAsDouble());
          outputs.appliedDeploySpeed = deploySpeed.getAsDouble();
          RobotState.setDeployMode(RobotState.DeployModeState.SPEED);
          stopDeployOnCurrentSpike = true;
        },
        () -> {
          System.out.println("deployWithSpeed: " + 0);
          outputs.appliedDeploySpeed = 0;
          RobotState.setDeployMode(RobotState.DeployModeState.OFF);
        });
  }

  public Command runIntake() {
    return runEnd(
            () -> {
              outputs.appliedIntakeSpeed = IntakeConstants.INTAKE_MOTOR_SPEED;
            },
            () -> {
              outputs.appliedIntakeSpeed = 0.0;
            });
  }
  public Command reverseIntake() {
    return runEnd(
            () -> {
              outputs.appliedIntakeSpeed = IntakeConstants.INTAKE_REVERSE_SPEED;
            },
            () -> {
              outputs.appliedIntakeSpeed = 0.0;
            });
  }

  // public Command deployAndIntake() {
  //   return deploy().alongWith(runIntake());
  // }

  public Command stopIntake() {
    return run(
        () -> {
          outputs.appliedIntakeSpeed = 0;
          outputs.appliedDeploySpeed = 0;
          RobotState.setDeployMode(DeployModeState.OFF);
        });
  }

  public void deployStop() {
    outputs.appliedDeploySpeed = 0;
    RobotState.setDeployMode(DeployModeState.OFF);
    isDeployStopped = true;
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

    Logger.recordOutput("Intake/Mode", RobotState.getDeployMode().toString());
    Logger.recordOutput("Intake/Applied Intake Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput("Intake/Applied Output Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput("Intake/Applied Deploy Current", outputs.appliedDeployCurrent);
    Logger.recordOutput("Intake/kP", outputs.kP);
    Logger.recordOutput("Intake/kI", outputs.kI);
    Logger.recordOutput("Intake/kD", outputs.kD);
    Logger.recordOutput("Intake/kFF", outputs.kFF);
    Logger.recordOutput("Intake/desiredPosition", outputs.desiredPosition);
    io.applyOutputs(outputs);

    // For testing, turn deploy motors off after hitting hard stop
    // TODO: Don't leave this in code as we will likely stop between bottom and top when hopper is
    // full
    if (stopMotorWhenNotMoving) {
      if (inputs.intakeLeftSpeed == 0) {
        stoppedLoopCount++;
      } else {
        stoppedLoopCount = 0;
      }
      if (stoppedLoopCount > STOPPED_LOOP_COUNT_NEEDED) {
        deployStop();
      }
    }

    if (stopDeployOnCurrentSpike
        && inputs.deploySupplyCurrent >= IntakeConstants.DEPLOY_CURRENT_STOP_THRESHOLD) {
      deployStop();
    }

    // Stop the deploy motor if our sensors detect we are down or up
    if ((outputs.desiredPosition == IntakeConstants.DOWN) && inputs.isDeployDown
        || (outputs.desiredPosition == IntakeConstants.UP) && inputs.isDeployUp) {
      deployStop();
    }
  }
}
