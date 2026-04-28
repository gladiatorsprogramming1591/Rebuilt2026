package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.KICKER_TABLE_KEY;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Controls the kicker motor that feeds fuel from the hopper into the shooter. */
public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputsAutoLogged outputs = new KickerIOOutputsAutoLogged();

  /**
   * Creates a kicker subsystem using the provided hardware implementation.
   *
   * @param io kicker hardware abstraction
   */
  public Kicker(KickerIO io) {
    this.io = io;
  }

  /** Updates kicker inputs, logs subsystem state, and applies requested outputs. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    Logger.recordOutput(KICKER_TABLE_KEY + "DesiredSpeed", outputs.desiredKickerSpeed);

    io.applyOutputs(outputs);
  }

  /**
   * Runs the kicker at the configured tunable speed.
   *
   * @return command that runs the kicker until interrupted
   */
  public Command runKickerMotor() {
    return runEnd(this::runKicker, this::stopKicker);
  }

  /**
   * Keeps the kicker stopped.
   *
   * <p>This is intended to be the kicker default command.
   *
   * @return command that continuously stops the kicker
   */
  public Command stopKickerMotor() {
    return run(this::stopKicker);
  }

  /** Requests kicker motor output at the configured tunable speed. */
  private void runKicker() {
    outputs.desiredKickerSpeed = KickerConstants.getKickerMotorSpeed();
  }

  /** Requests zero kicker motor output. */
  private void stopKicker() {
    outputs.desiredKickerSpeed = 0.0;
  }
}
