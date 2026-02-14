package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  public Roller(RollerIO io) {
    this.io = io;
  }

  public Command runRollerMotor() {
    return runEnd(
        () -> {
          io.setRollerSpeed(RollerConstants.ROLLER_MOTOR_SPEED);
        },
        () -> {
          io.setRollerSpeed(0);
        });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }
}
