package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodIO.HoodMode;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputsAutoLogged outputs = new HoodIOOutputsAutoLogged();

  private boolean hasBeenZeroed = false;

  private static final LoggedTunableNumber goalPosition =
      new LoggedTunableNumber("Hood/GoalPosition", 100.0);
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
          io.stopHood();
        });
  }

  public Command runHoodDown() {
    return runEnd(
        () -> {
          outputs.mode = HoodMode.SPEED;
          io.setHoodSpeed(HoodConstants.HOOD_DOWN_SPEED);
        },
        () -> {
          io.stopHood();
        });
  }

  public Command runHoodPosition(DoubleSupplier angleSupplier) {
    return run(
        () -> {
          if (hasBeenZeroed) {
            outputs.mode = HoodMode.POSITION;
            // io.setHoodPosition(angle);
            outputs.desiredHoodAngle = angleSupplier.getAsDouble();
          }
        });
  }

  public Command runHoodTarget() {
    return run(
        () -> {
          boolean tune = false;
          // boolean tune = Constants.tuningMode;
          if (tune) {
            outputs.mode = HoodMode.POSITION;
            outputs.desiredHoodAngle = goalPosition.getAsDouble();
          } else {
            outputs.mode = HoodMode.POSITION;
            var params = ShooterCalculation.getInstance().getParameters();
            outputs.desiredHoodAngle = params.hoodAngle();
            // outputs.velocityRadPerSecond = params.hoodVelocity();
          }
        });
  }

  public Command stopHood() {
    return runOnce(() -> io.stopHood());
  }

  public Command runHoodToZero() {
    outputs.mode = HoodMode.SPEED;
    outputs.desiredHoodAngle = 0;
    return runOnce(() -> io.setHoodCurrentLimit(HoodConstants.HOOD_ZEROING_CURRENT_LIMIT))
        .andThen(
            run(() -> io.runHoodToZero())
                .until(() -> io.isHoodAtTrueZero())
                .andThen(() -> io.zeroHood()))
        .finallyDo(
            () -> {
              io.stopHood();
              io.setHoodCurrentLimit(HoodConstants.HOOD_CURRENT_LIMIT);
              io.resetHoodTimer();
            });
  }

  public BooleanSupplier isHoodAtAngle() {
    return () -> (outputs.desiredHoodAngle - inputs.hoodAngle) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  public void periodic() {
    io.updateInputs(inputs);
    hasBeenZeroed = true; // || -50 < inputs.hoodAngle && inputs.hoodAngle < ;
    Logger.processInputs("Hood", inputs);
    outputs.kP = kP.get();
    outputs.kD = kD.get();
    outputs.kS = kS.get();

    Logger.recordOutput("Hood/Desired Hood Angle", outputs.desiredHoodAngle);
    Logger.recordOutput("Hood/Has Been Zeroed", hasBeenZeroed);
    Logger.recordOutput("Hood/Mood Mode", outputs.mode);
    SmartDashboard.putBoolean("Hood-hasBeenZeroed", hasBeenZeroed);

    if (this.hasBeenZeroed) {
      io.applyOutputs(outputs);
    } else {
      io.runHoodToZero();
    }
  }
}
