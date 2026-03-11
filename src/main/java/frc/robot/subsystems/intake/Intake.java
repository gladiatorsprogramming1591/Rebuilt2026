package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
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
        },
        () -> {
          io.stopDeployMotor();
        });
  }

  public Command deployAndIntake() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.DEPLOY_SPEED);
          io.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
        },
        () -> {
          io.stopDeployMotor();
          io.setIntakeSpeed(IntakeConstants.INTAKE_IDLE_SPEED);
        });
  }

  public Command startIntakeMotor() {
    return new InstantCommand(() -> io.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED));
  }

  public Command idleIntakeMotor() {
    return new InstantCommand(() -> io.setIntakeSpeed(IntakeConstants.INTAKE_IDLE_SPEED));
  }

  public Command stopIntakeMotor() {
    return new RunCommand(() -> io.setIntakeSpeed(0.0), this);
  }

  public Command deployIntakeUsingCurrent() {
    return runEnd(
        () -> {
          io.setDeployTorqueCurrentFOC(IntakeConstants.DEPLOY_TORQUE_CURRENT);
        },
        () -> {
          io.stopDeployMotor();
        });
  }

  public Command stowIntakeUsingCurrent() {
    return runEnd(
        () -> {
          io.setDeployTorqueCurrentFOC(-IntakeConstants.DEPLOY_TORQUE_CURRENT);
        },
        () -> {
          io.stopDeployMotor();
        });
  }

  public Command runStow() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.STOW_SPEED);
        },
        () -> {
          io.stopDeployMotor();
        });
  }

  public Command runIntakeMotor() {
    return runEnd(
        () -> {
          io.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
        },
        () -> {
          io.stopIntakeMotor();
        });
  }

  public Command stow() {
    return runEnd(
        () -> {
          io.setDeploySpeed(IntakeConstants.STOW_SPEED);
        },
        () -> {
          io.stopDeployMotor();
        });
  }

  public Command deployIntakeOn() {
    return deployIntake()
        .alongWith(
            new WaitCommand(IntakeConstants.INTAKE_DELAY_SECONDS).andThen(startIntakeMotor()));
  }

  public Command stowIntakeOff() {
    return stopIntakeMotor().andThen(stow());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake speed", inputs);
  }
}
