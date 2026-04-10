package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  // Deploy tunable configs
  private static final LoggedTunableNumber kdeployP =
      new LoggedTunableNumber("Intake/kdeployP", IntakeConstants.DeployConfigs.kP);
  private static final LoggedTunableNumber kdeployI =
      new LoggedTunableNumber("Intake/kdeployI", IntakeConstants.DeployConfigs.kI);
  private static final LoggedTunableNumber kdeployD =
      new LoggedTunableNumber("Intake/kdeployD", IntakeConstants.DeployConfigs.kD);
  private static final LoggedTunableNumber kdeployFF =
      new LoggedTunableNumber("Intake/kdeployFF", IntakeConstants.DeployConfigs.kFF);
  // Stow tunable configs
  private static final LoggedTunableNumber kstowP =
      new LoggedTunableNumber("Intake/kstowP", IntakeConstants.StowConfigs.kP);
  private static final LoggedTunableNumber kstowI =
      new LoggedTunableNumber("Intake/kstowI", IntakeConstants.StowConfigs.kI);
  private static final LoggedTunableNumber kstowD =
      new LoggedTunableNumber("Intake/kstowD", IntakeConstants.StowConfigs.kD);
  private static final LoggedTunableNumber kstowFF =
      new LoggedTunableNumber("Intake/kstowFF", IntakeConstants.StowConfigs.kFF);
  private static final LoggedTunableNumber kstowMMAcceleration =
      new LoggedTunableNumber("Intake/kstowMMAcceleration", IntakeConstants.StowConfigs.kmmAcceleration);
  private static final LoggedTunableNumber kstowMMJerk =
      new LoggedTunableNumber("Intake/kstowMMJerk", IntakeConstants.StowConfigs.kmmJerk);

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
              RobotState.setDeployMode(RobotState.DeployModeState.DEPLOY_POSITION);
              stopDeployOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedDeploySpeed = 0;
              RobotState.setDeployMode(RobotState.DeployModeState.OFF);
            })
        .until(() -> isDeployStopped);
  }

  // TODO: Change from runEnd to let applyOutputs check for up and stop motor. Probably need new states.
  public Command stow() {
    return runEnd(
            () -> {
              isDeployStopped = false;
              outputs.desiredPosition = IntakeConstants.UP;
              RobotState.setDeployMode(RobotState.DeployModeState.STOW_POSITION);
              stopDeployOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedDeploySpeed = 0;
              RobotState.setDeployMode(RobotState.DeployModeState.OFF);
            })
        .until(() -> isDeployStopped);
  }

  public Command stowBump() {
    return runEnd(
            () -> {
              isDeployStopped = false;
              outputs.desiredPosition = IntakeConstants.BUMP;
              RobotState.setDeployMode(RobotState.DeployModeState.BUMP_POSITION);
              stopDeployOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedDeploySpeed = 0;
              RobotState.setDeployMode(RobotState.DeployModeState.OFF);
            });
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
    // Deploy configs
    outputs.kdeployP = kdeployP.getAsDouble();
    outputs.kdeployI = kdeployI.getAsDouble();
    outputs.kdeployD = kdeployD.getAsDouble();
    outputs.kdeployFF = kdeployFF.getAsDouble();
    // Stow configs
    outputs.kstowP = kstowP.getAsDouble();
    outputs.kstowI = kstowI.getAsDouble();
    outputs.kstowD = kstowD.getAsDouble();
    outputs.kstowFF = kstowFF.getAsDouble();
    outputs.kstowMMAcceleration = kstowMMAcceleration.getAsDouble();
    outputs.kstowMMJerk = kstowMMJerk.getAsDouble();

    Logger.recordOutput("Intake/Mode", RobotState.getDeployMode().toString());
    Logger.recordOutput("Intake/Applied Intake Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput("Intake/Applied Output Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput("Intake/Applied Deploy Current", outputs.appliedDeployCurrent);
    // Deploy configs
    Logger.recordOutput("Intake/kdeployP", outputs.kdeployP);
    Logger.recordOutput("Intake/kdeployI", outputs.kdeployI);
    Logger.recordOutput("Intake/kdeployD", outputs.kdeployD);
    Logger.recordOutput("Intake/kdeployFF", outputs.kdeployFF);
    // Stow configs
    Logger.recordOutput("Intake/kstowP", outputs.kstowP);
    Logger.recordOutput("Intake/kstowI", outputs.kstowI);
    Logger.recordOutput("Intake/kstowD", outputs.kstowD);
    Logger.recordOutput("Intake/kstowFF", outputs.kstowFF);
    Logger.recordOutput("Intake/kstowMMAcceleration", outputs.kstowMMAcceleration);
    Logger.recordOutput("Intake/kstowMMJerk", outputs.kstowMMJerk);

    Logger.recordOutput("Intake/desiredPosition", outputs.desiredPosition);
    io.applyOutputs(outputs);

    // For testing, turn deploy motors off after hitting hard stop
    // TODO: Don't leave this in code as we will likely stop between bottom and top when hopper is full
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
