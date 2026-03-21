package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
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

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Command deployIntake() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.DEPLOY_SPEED);
          outputs.appliedDeploySpeed = IntakeConstants.DEPLOY_SPEED;
          outputs.appliedDeployCurrent = 0;
        },
        () -> {
          io.stopDeployMotor();
          outputs.appliedDeploySpeed = 0;
          outputs.appliedDeployCurrent = 0;
        });
  }

  public Command deployAndIntake(boolean idleIntake) {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.DEPLOY_SPEED);
          io.setIntakeSpeed(
              idleIntake ? IntakeConstants.INTAKE_IDLE_SPEED : IntakeConstants.INTAKE_MOTOR_SPEED);
          outputs.appliedDeploySpeed = IntakeConstants.DEPLOY_SPEED;
          outputs.appliedDeployCurrent = 0;
          outputs.appliedIntakeSpeed =
              idleIntake ? IntakeConstants.INTAKE_IDLE_SPEED : IntakeConstants.INTAKE_MOTOR_SPEED;
        },
        () -> {
          io.stopDeployMotor();
          io.setIntakeSpeed(IntakeConstants.INTAKE_IDLE_SPEED);
          outputs.appliedDeploySpeed = 0;
          outputs.appliedDeployCurrent = 0;
          outputs.appliedIntakeSpeed = IntakeConstants.INTAKE_IDLE_SPEED;
        });
  }

  public Command idleIntakeMotorInstant() {
    outputs.appliedIntakeSpeed = IntakeConstants.INTAKE_IDLE_SPEED;
    return new InstantCommand(() -> io.setIntakeSpeed(IntakeConstants.INTAKE_IDLE_SPEED));
  }

  public Command stopIntakeMotor() {
    outputs.appliedIntakeSpeed = 0;
    return new RunCommand(() -> io.stopIntakeMotor(), this);
  }

  public Command deployIntakeUsingCurrent() {
    return runEnd(
        () -> {
          io.setDeployTorqueCurrentFOC(IntakeConstants.DEPLOY_TORQUE_CURRENT);
          outputs.appliedDeployCurrent = IntakeConstants.DEPLOY_TORQUE_CURRENT;
          outputs.appliedDeploySpeed = 0;
        },
        () -> {
          io.stopDeployMotor();
          outputs.appliedDeployCurrent = 0;
          outputs.appliedDeploySpeed = 0;
        });
  }

  public Command stowIntakeUsingCurrent() {
    return runEnd(
        () -> {
          io.setDeployTorqueCurrentFOC(-IntakeConstants.DEPLOY_TORQUE_CURRENT);
          outputs.appliedDeployCurrent = -IntakeConstants.DEPLOY_TORQUE_CURRENT;
          outputs.appliedDeploySpeed = 0;
        },
        () -> {
          io.stopDeployMotor();
          outputs.appliedDeployCurrent = 0;
          outputs.appliedDeploySpeed = 0;
        });
  }

  public Command runStow() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.STOW_SPEED);
          outputs.appliedDeploySpeed = IntakeConstants.STOW_SPEED;
          outputs.appliedDeployCurrent = 0;
        },
        () -> {
          io.stopDeployMotor();
          outputs.appliedDeployCurrent = 0;
          outputs.appliedDeploySpeed = 0;
        });
  }

  public Command runIntakeMotor() {
    return runEnd(
        () -> {
          outputs.appliedIntakeSpeed = IntakeConstants.INTAKE_MOTOR_SPEED;
          io.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
        },
        () -> {
          outputs.appliedIntakeSpeed = 0;
          io.stopIntakeMotor();
        });
  }

  // TODO: apply outputs?
  public Command reverseIntakeMotor() {
    return runEnd(
        () -> {
          io.setIntakeSpeed(-IntakeConstants.INTAKE_MOTOR_SPEED);
        },
        () -> {
          io.stopIntakeMotor();
        });
  }

  public Command stow() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.STOW_SPEED);
          outputs.appliedDeployCurrent = 0;
          outputs.appliedDeploySpeed = IntakeConstants.STOW_SPEED;
        },
        () -> {
          io.stopDeployMotor();
          outputs.appliedDeployCurrent = 0;
          outputs.appliedDeploySpeed = 0;
        });
  }

  public Command stowIntakeOff() {
    outputs.appliedDeployCurrent = 0;
    outputs.appliedDeploySpeed = 0;
    outputs.appliedIntakeSpeed = 0;
    return stopIntakeMotor().andThen(stow());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake speed", inputs);
    Logger.recordOutput("Intake/Applied Intake Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput("Intake/Applied Output Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput("Intake/Applied Deploy Current", outputs.appliedDeployCurrent);
    io.applyOutputs(outputs);
  }
}
