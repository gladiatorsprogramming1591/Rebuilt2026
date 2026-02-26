package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    this.io = io;
  }

  public Command runKickerMotor() {
    return runEnd(
        () -> {
          io.setKickerSpeed(KickerConstants.KICKER_MOTOR_SPEED);
        },
        () -> {
          io.setKickerSpeed(0.0);
        });
  }

  public Command startKickerMotor() {
    return new InstantCommand(() -> io.setKickerSpeed(KickerConstants.KICKER_MOTOR_SPEED), this);
  }

  public Command stopKickerMotor() {
    return new InstantCommand(() -> io.setKickerSpeed(0.0), this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake speed", inputs);
  }
}
