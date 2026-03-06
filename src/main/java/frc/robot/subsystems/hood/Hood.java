package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();


  private static final LoggedTunableNumber goalPosition = 
      new LoggedTunableNumber("Hood/GoalPosition", 0.0);
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Hood/kP", HoodConstants.kP);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Hood/kD", HoodConstants.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Hood/kS", HoodConstants.kS);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  public Hood(HoodIO io) {
    this.io = io;
  }

  public Command runHoodUp() {
    return runEnd(
        () -> {
          io.setHoodSpeed(HoodConstants.HOOD_UP_SPEED);
        },
        () -> {
          io.setHoodSpeed(0.0);
        });
  }

  public Command runHoodDown() {
    return runEnd(
        () -> {
          io.setHoodSpeed(HoodConstants.HOOD_DOWN_SPEED);
        },
        () -> {
          io.setHoodSpeed(0.0);
        });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

  }
}
