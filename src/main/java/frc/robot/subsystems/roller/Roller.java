package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.spindexer.RollerIO.RollerIOInputs;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputs inputs = new RollerIOInputs();

  public Roller(RollerIO io) {
    this.io = io;
  }

  public void setSpeed(CommandXboxController operator_controller) {
    double speed =
        RollerConstants.ROLLER_MAX_SPEED
            * (operator_controller.getRightTriggerAxis()
                - operator_controller.getLeftTriggerAxis());
    io.setSpeed(speed);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Roller speed", inputs.speed);
  }
}
