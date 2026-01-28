package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOInputs;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputs inputs = new SpindexerIOInputs();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void setSpeed(CommandXboxController operator_controller) {
    double speed =
        SpindexerConstants.SPINDEXER_MAX_SPEED
            * (operator_controller.getRightTriggerAxis()
                - operator_controller.getLeftTriggerAxis());
    io.setSpeed(speed);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Spindexer speed", inputs.speed);
  }
}
