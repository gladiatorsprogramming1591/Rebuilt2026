package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.DeployModeState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.intake.IntakeConstants.kdeployTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowTableKey;

/**
 * Name conventions:
 * <ul>
 *  <li><b>Intake:</b> Whole subsystem
 *  <li><b>Slapdown:</b> Intake pivoting arm
 *    <ul>
 *      <li> <b>Deploy:</b> Full extension
 *      <li> <b>Stow:</b> Full retraction into frame perimeter
 *    </ul>
 *  <li><b>Rollers:</b> Rotating tubes that propel fuel into hopper
 * </ul>
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();
  
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
    outputs.desiredPosition = MathUtil.clamp(goalAngle, IntakeConstants.intakeMinAngle, IntakeConstants.intakeMaxAngle);
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
          System.out.println("deployWithSpeed: " + IntakeConstants.deploySpeed.getAsDouble());
          outputs.appliedDeploySpeed = IntakeConstants.deploySpeed.getAsDouble();
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
    outputs.kdeployP = IntakeConstants.kdeployP.getAsDouble();
    outputs.kdeployI = IntakeConstants.kdeployI.getAsDouble();
    outputs.kdeployD = IntakeConstants.kdeployD.getAsDouble();
    outputs.kdeployFF = IntakeConstants.kdeployFF.getAsDouble();
    // Stow configs
    outputs.kstowP = IntakeConstants.kstowP.getAsDouble();
    outputs.kstowI = IntakeConstants.kstowI.getAsDouble();
    outputs.kstowD = IntakeConstants.kstowD.getAsDouble();
    outputs.kstowFF = IntakeConstants.kstowFF.getAsDouble();
    outputs.kstowMMAcceleration = IntakeConstants.kstowMMAcceleration.getAsDouble();
    outputs.kstowMMJerk = IntakeConstants.kstowMMJerk.getAsDouble();

    Logger.recordOutput(kintakeTableKey + "Mode", RobotState.getDeployMode().toString());
    Logger.recordOutput(kintakeTableKey + "Applied Roller Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput(kintakeTableKey + "Applied Slapdown Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput(kintakeTableKey + "Applied Slapdown Current", outputs.appliedDeployCurrent);
    // Deploy configs
    Logger.recordOutput(kdeployTableKey + "kdeployP", outputs.kdeployP);
    Logger.recordOutput(kdeployTableKey + "kdeployI", outputs.kdeployI);
    Logger.recordOutput(kdeployTableKey + "kdeployD", outputs.kdeployD);
    Logger.recordOutput(kdeployTableKey + "kdeployFF", outputs.kdeployFF);
    // Stow configs
    Logger.recordOutput(kstowTableKey + "kstowP", outputs.kstowP);
    Logger.recordOutput(kstowTableKey + "kstowI", outputs.kstowI);
    Logger.recordOutput(kstowTableKey + "kstowD", outputs.kstowD);
    Logger.recordOutput(kstowTableKey + "kstowFF", outputs.kstowFF);
    Logger.recordOutput(kstowTableKey + "kstowMMAcceleration", outputs.kstowMMAcceleration);
    Logger.recordOutput(kstowTableKey + "kstowMMJerk", outputs.kstowMMJerk);

    Logger.recordOutput(kintakeTableKey + "desiredPosition", outputs.desiredPosition);
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
