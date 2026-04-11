package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputsAutoLogged outputs = new KickerIOOutputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public Command runKickerMotor() {
    return runEnd(
        () -> {
          outputs.desiredKickerSpeed = KickerConstants.KICKER_MOTOR_SPEED;
        },
        () -> {
          outputs.desiredKickerSpeed = 0.0;
        });
  }

  public Command startKickerMotor() {
    return new RunCommand(
        () -> outputs.desiredKickerSpeed = KickerConstants.KICKER_MOTOR_SPEED, this);
  }

  public Command stopKickerMotor() {
    return new RunCommand(() -> outputs.desiredKickerSpeed = 0.0, this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    // io.applyOutputs(outputs); // TODO TEMP
    Logger.recordOutput("Kicker/Speed", outputs.desiredKickerSpeed);
  }
}
