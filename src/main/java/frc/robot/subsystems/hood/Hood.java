package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
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

  private boolean hasInitiallyBeenZeroed = true; // TODO: set to false when fixed
  private boolean hasReducedCurrentLimit = false;

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

  public BooleanSupplier getHasInitiallyBeenZeroed() {
    return () -> hasInitiallyBeenZeroed;
  }

  public Command runHoodUp() {
    return runEnd(
        () -> {
          outputs.mode = HoodMode.SPEED;
          outputs.desiredHoodSpeed = HoodConstants.HOOD_UP_SPEED;
        },
        () -> {
          outputs.desiredHoodSpeed = 0;
        });
  }

  public Command runHoodDown() {
    return runEnd(
        () -> {
          outputs.mode = HoodMode.SPEED;
          outputs.desiredHoodSpeed = HoodConstants.HOOD_DOWN_SPEED;
        },
        () -> {
          outputs.desiredHoodSpeed = 0;
        });
  }

  public Command runHoodPosition(DoubleSupplier angleSupplier) {
    return run(
        () -> {
          if (hasInitiallyBeenZeroed) {
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

  public Command stopHoodOnce() {
    return runOnce(
        () -> {
          outputs.mode = HoodMode.SPEED;
          outputs.desiredHoodSpeed = 0;
        });
  }

  public Command stopHoodContinuously()
  {
    return run(
        () -> {
          outputs.mode = HoodMode.SPEED;
          outputs.desiredHoodSpeed = 0;
        });
  }

  public Command runHoodToZero() {
    return new ConditionalCommand(
      // On true
      stopHoodContinuously(), // Indefinitely stops hood until inturrupted
      // stopHoodOnce(),
      // On false
      new SequentialCommandGroup(
        runOnce(() -> 
          {
            outputs.mode = HoodMode.SPEED;
            setReducedStatorCurrentLimit();
            outputs.desiredHoodAngle = 0;
            outputs.desiredHoodSpeed = HoodConstants.HOOD_ZEROING_SPEED;
          }
        ),
        idle() // Waits as desiredHoodSpeed set above lowers hood
          .until(() -> io.isHoodAtTrueZero())
          .andThen(() -> io.zeroHood())
      ).finallyDo(() ->
        {
          stopHoodOnce();
          setDefaultStatorCurrentLimit();
          io.resetHoodTimer();
        }
      ),
      // Condition
      io.isHoodWithinZeroTolerance());
  }

  public BooleanSupplier isHoodAtAngle() {
    return () -> (outputs.desiredHoodAngle - inputs.hoodAngle) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  public Command ZeroHood() {
    return new InstantCommand(() -> io.zeroHood());
  }

  private void setReducedStatorCurrentLimit()
  {
    if (!hasReducedCurrentLimit) {
      hasReducedCurrentLimit = true;
      io.setStatorCurrentLimit(HoodConstants.HOOD_ZEROING_STATOR_CURRENT_LIMIT);
    }
  }
  private void setDefaultStatorCurrentLimit()
  {
    if (hasReducedCurrentLimit) {
      hasReducedCurrentLimit = false;
      io.setStatorCurrentLimit(HoodConstants.HOOD_STATOR_CURRENT_LIMIT);
    }
  }

  private boolean inTrenchBounds() {
    Translation2d robotTranslation = RobotState.getInstance().getEstimatedPose().getTranslation();
    return HoodTrenchBounds.redNearTrench.contains(robotTranslation)
        || HoodTrenchBounds.redFarTrench.contains(robotTranslation)
        || HoodTrenchBounds.blueNearTrench.contains(robotTranslation)
        || HoodTrenchBounds.blueFarTrench.contains(robotTranslation);
  }

  public void periodic() {
    io.updateInputs(inputs);
    hasInitiallyBeenZeroed = hasInitiallyBeenZeroed || DriverStation.isEnabled() && io.isHoodAtTrueZero();
    Logger.processInputs("Hood", inputs);
    outputs.kP = kP.get();
    outputs.kD = kD.get();
    outputs.kS = kS.get();

    Logger.recordOutput("Hood/Desired Hood Angle", outputs.desiredHoodAngle);
    Logger.recordOutput("Hood/Has Been Zeroed", hasInitiallyBeenZeroed);
    Logger.recordOutput("Hood/Hood Mode", outputs.mode);
    Logger.recordOutput("Hood/Desired Hood Speed", outputs.desiredHoodSpeed);
    SmartDashboard.putBoolean("Hood-hasBeenZeroed", hasInitiallyBeenZeroed);
    SmartDashboard.putBoolean("Hood-hasReducedCurrentLimit", hasReducedCurrentLimit);
    SmartDashboard.putBoolean("Hood-isHoodWithinZeroTolerance", io.isHoodWithinZeroTolerance().getAsBoolean());

    if (hasInitiallyBeenZeroed) {
      runOnce(() -> setDefaultStatorCurrentLimit());
      io.applyOutputs(outputs);
    } else {
      runOnce(() -> setReducedStatorCurrentLimit());
      io.runHoodToZero();
    }
  }
}
