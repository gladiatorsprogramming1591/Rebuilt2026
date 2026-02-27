package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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

  public Command deployIntakeUsingCurrent() {
    return runEnd(
        () -> {
          io.setDeployTorqueCurrentFOC(IntakeConstants.DEPLOY_TORQUE_CURRENT);
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

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake speed", inputs);
  }
}
