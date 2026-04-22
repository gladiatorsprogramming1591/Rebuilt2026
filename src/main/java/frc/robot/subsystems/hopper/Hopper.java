package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private final HopperIOOutputsAutoLogged outputs = new HopperIOOutputsAutoLogged();

  private final Timer hopperEmptyTimer = new Timer();
  private boolean isHopperEmptyOverTime = false;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  public Command runBeltMotor() {
    return runEnd(
        () -> {
          outputs.beltSpeed = -HopperConstants.BELT_MOTOR_SPEED;
        },
        () -> {
          outputs.beltSpeed = 0;
        });
  }

  public Command startBeltMotors() {
    return runEnd(
        () -> {
          outputs.beltSpeed = -HopperConstants.BELT_MOTOR_SPEED;
        },
        () -> {
          outputs.beltSpeed = 0;
        });
  }

  public Command runBeltWhileIntaking() {
    return runEnd(
        () -> {
          outputs.useBeltWhileIntakeCurrent = true;
          outputs.beltSpeed = -HopperConstants.BELT_MOTOR_SPEED;
        },
        () -> {
          outputs.useBeltWhileIntakeCurrent = false;
          outputs.beltSpeed = 0;
        });
  }

  public Command reverseBeltMotors() {
    return runEnd(
        () -> {
          outputs.beltSpeed = HopperConstants.BELT_MOTOR_SPEED;
        },
        () -> {
          outputs.beltSpeed = 0;
        });
  }

  public Command stopBeltMotors() {
    return new RunCommand(
        () -> {
          outputs.beltSpeed = 0;
        },
        this);
  }

  public boolean isHopperEmpty() {
    return isHopperEmptyOverTime;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    Logger.recordOutput("Hopper/Belt Speed", outputs.beltSpeed);
    Logger.recordOutput("Hopper/useBeltWhileIntakeCurrent", outputs.useBeltWhileIntakeCurrent);
    Logger.recordOutput("Hopper/usingLowerCurrent", outputs.usingLowerCurrent);
    io.applyOutputs(outputs);
    if (inputs.hopperEmpty) {
      if (!hopperEmptyTimer.isRunning()) {
        hopperEmptyTimer.start();
      }
    } else {
      hopperEmptyTimer.stop();
      hopperEmptyTimer.reset();
    }
    isHopperEmptyOverTime = hopperEmptyTimer.get() >= HopperConstants.MIN_EMPTY_DURATION;
    Logger.recordOutput("Hopper/isHopperEmptyOverTime", isHopperEmptyOverTime);
  }
}
