package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputsAutoLogged outputs = new RollerIOOutputsAutoLogged();

  public Roller(RollerIO io) {
    this.io = io;
  }

  public Command runTopRollerMotor() {
    return runEnd(
        () -> {
          // io.setTopRollerSpeed(RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          outputs.topRollerSpeed = -RollerConstants.TOP_ROLLER_MOTOR_SPEED;
        },
        () -> {
          // io.setTopRollerSpeed(0);
          outputs.topRollerSpeed = 0;
        });
  }

  public Command runBottomRollerMotor() {
    return runEnd(
        () -> {
          outputs.bottomRollerSpeed = -RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED;
          // io.setBottomRollerSpeed(-RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        () -> {
          outputs.bottomRollerSpeed = 0;
          // io.setBottomRollerSpeed(0);
        });
  }

  public Command startRollerMotors() {
    return runEnd(
        () -> {
          outputs.topRollerSpeed = RollerConstants.TOP_ROLLER_MOTOR_SPEED;
          outputs.bottomRollerSpeed = -RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED;
          // io.setTopRollerSpeed(RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          // io.setBottomRollerSpeed(-RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        () -> {
          outputs.topRollerSpeed = 0;
          outputs.bottomRollerSpeed = 0;
        });
  }

  public Command runBottomRollerWhileIntaking() {
    return runEnd(
        () -> {
          outputs.useRollerWhileIntakeCurrent = true;
          outputs.topRollerSpeed = 0;
          outputs.bottomRollerSpeed = -RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED;
          // io.setTopRollerSpeed(RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          // io.setBottomRollerSpeed(-RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        () -> {
          outputs.useRollerWhileIntakeCurrent = false;
          outputs.topRollerSpeed = 0;
          outputs.bottomRollerSpeed = 0;
        });
  }

  public Command reverseRollerMotors() {
    return runEnd(
        () -> {
          outputs.topRollerSpeed = -RollerConstants.TOP_ROLLER_MOTOR_SPEED;
          outputs.bottomRollerSpeed = RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED;
          // io.setTopRollerSpeed(-RollerConstants.TOP_ROLLER_MOTOR_SPEED);
          // io.setBottomRollerSpeed(RollerConstants.BOTTOM_ROLLER_MOTOR_SPEED);
        },
        () -> {
          outputs.topRollerSpeed = 0;
          outputs.bottomRollerSpeed = 0;
        });
  }

  public Command stopRollerMotors() {
    return new RunCommand(
        () -> {
          outputs.topRollerSpeed = 0;
          outputs.bottomRollerSpeed = 0;
          // io.setTopRollerSpeed(0.0);
          // io.setBottomRollerSpeed(0.0);
        },
        this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

    Logger.recordOutput("Roller/Top Roller Speed", outputs.topRollerSpeed);
    Logger.recordOutput("Roller/Bottom Roller Speed", outputs.bottomRollerSpeed);
    Logger.recordOutput("Roller/useRollerWhileIntakeCurrent", outputs.useRollerWhileIntakeCurrent);
    Logger.recordOutput("Roller/usingLowerCurrent", outputs.usingLowerCurrent);
    // SmartDashboard.putNumber("Top Roller Speed", outputs.topRollerSpeed);
    // SmartDashboard.putNumber("Bottom Roller Speed", outputs.bottomRollerSpeed);
    io.applyOutputs(outputs);
  }
}
