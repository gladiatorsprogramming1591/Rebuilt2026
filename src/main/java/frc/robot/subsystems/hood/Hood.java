package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import frc.robot.subsystems.hood.HoodIO.HoodMode;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private boolean hasBeenZeroed = false;

  private static final LoggedTunableNumber goalPosition =
      new LoggedTunableNumber("Hood/GoalPosition", 500.0);
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
          outputs.mode = HoodMode.SPEED;
          io.setHoodSpeed(HoodConstants.HOOD_UP_SPEED);
        },
        () -> {
          io.setHoodSpeed(0.0);
        });
  }

  public Command runHoodDown() {
    return runEnd(
        () -> {
          outputs.mode = HoodMode.SPEED;
          io.setHoodSpeed(HoodConstants.HOOD_DOWN_SPEED);
        },
        () -> {
          io.setHoodSpeed(0.0);
        });
  }

  public Command runHoodPosition(double angle) {
    return runEnd(
        () -> {
          outputs.mode = HoodMode.POSITION;
          io.setHoodPosition(angle * 8);
        },
        () -> {
          io.setHoodSpeed(0.0);
        });
  }

  public Command runHoodTarget() {
    return run(
        () -> {
          outputs.mode = HoodMode.POSITION;
          var params = ShooterCalculation.getInstance().getParameters();
          outputs.desiredHoodAngle = params.hoodAngle();
          outputs.velocityRadPerSecond = params.hoodVelocity();
        });
  }

  public Command runHoodToZero() {
    outputs.mode = HoodMode.SPEED;
    return new RunCommand(() -> io.driveHoodToZero())
        .until(() -> io.isHoodAtTrueZero())
        .andThen(() -> io.zero())
        .finallyDo(
            () -> {
              io.setHoodSpeed(0.0);
              io.setHoodCurrentLimit(HoodConstants.HOOD_CURRENT_LIMIT);
              io.resetHoodZeroTimer();
            });
  }

  public void periodic() {
    io.updateInputs(inputs);
    hasBeenZeroed = hasBeenZeroed || -0.5 < inputs.hoodAngle && inputs.hoodAngle < 0.5;
    Logger.processInputs("Hood", inputs);
    io.applyOutputs(outputs);
  }
}
