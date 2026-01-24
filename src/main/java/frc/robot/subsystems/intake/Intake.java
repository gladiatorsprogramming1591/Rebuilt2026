package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setSpeed(CommandXboxController controller) {
    double speed =
        IntakeConstants.INTAKE_MAX_SPEED
            * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
    io.setSpeed(speed);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Intake speed", inputs.speed);
  }
}
