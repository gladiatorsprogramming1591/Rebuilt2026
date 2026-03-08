package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Roller(RollerIO io) {
    this.io = io;
  }

  public Command runTopRollerMotor() {
    return runEnd(
        () -> {
          io.setTopRollerSpeed(RollerConstants.TOP_ROLLER_MOTOR_SPEED);
        },
        () -> {
          io.setTopRollerSpeed(0);
        });
  }

  public Command runBottomRollerMotor() {
    return runEnd(
        () -> {
          io.setBottomRollerSpeed(-RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        () -> {
          io.setBottomRollerSpeed(0);
        });
  }

  public Command startRollerMotors() {
    return new RunCommand(
        () -> {
          io.setTopRollerSpeed(RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          io.setBottomRollerSpeed(-RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        this);
  }

  public Command reverseRollerMotors() {
    return new RunCommand(
        () -> {
          io.setTopRollerSpeed(-RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          io.setBottomRollerSpeed(RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        this);
  }

  public Command stopRollerMotors() {
    return new RunCommand(
        () -> {
          io.setTopRollerSpeed(0.0);
          io.setBottomRollerSpeed(0.0);
        },
        this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }
}
