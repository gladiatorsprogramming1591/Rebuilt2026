package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.HoodIO.HoodMode;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.util.LoggedTunableNumber;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_TABLE_KEY;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputsAutoLogged outputs = new HoodIOOutputsAutoLogged();

  private boolean hasInitiallyBeenZeroed = false; // TODO: set to false when fixed. Currently not beeing updated.
  private boolean hasReducedCurrentLimit = false;
  private boolean hasAppliedZeroAtCurrentHardStop = false;

  private static final LoggedTunableNumber goalPosition =
      new LoggedTunableNumber("Hood/GoalPosition", 100.0, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKP =
    new LoggedTunableNumber("Hood/Up/kP", HoodConstants.HOOD_UP_KP, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKI =
      new LoggedTunableNumber("Hood/Up/kI", HoodConstants.HOOD_UP_KI, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKD =
      new LoggedTunableNumber("Hood/Up/kD", HoodConstants.HOOD_UP_KD, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber upKS =
      new LoggedTunableNumber("Hood/Up/kS", HoodConstants.HOOD_UP_KS, Constants.Tuning.HOOD);

  private static final LoggedTunableNumber downKP =
      new LoggedTunableNumber("Hood/Down/kP", HoodConstants.HOOD_DOWN_KP, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKI =
      new LoggedTunableNumber("Hood/Down/kI", HoodConstants.HOOD_DOWN_KI, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKD =
      new LoggedTunableNumber("Hood/Down/kD", HoodConstants.HOOD_DOWN_KD, Constants.Tuning.HOOD);
  private static final LoggedTunableNumber downKS =
      new LoggedTunableNumber("Hood/Down/kS", HoodConstants.HOOD_DOWN_KS, Constants.Tuning.HOOD);
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

  // public Command runHoodToZero() {
  //   return new ConditionalCommand(
  //     // On true
  //     // stopHoodContinuously(), // Indefinitely stops hood until inturrupted
  //     stopHoodOnce(),
  //     // On false
  //     new SequentialCommandGroup(
  //       runOnce(() -> 
  //         {
  //           outputs.mode = HoodMode.SPEED;
  //           setReducedStatorCurrentLimit();
  //           outputs.desiredHoodAngle = 0;
  //           outputs.desiredHoodSpeed = HoodConstants.HOOD_ZEROING_SPEED;
  //         }
  //       ),
  //       idle() // Waits as desiredHoodSpeed set above lowers hood
  //         .until(() -> io.isHoodAtTrueZero())
  //         .andThen(() -> io.zeroHood())
  //     ).finallyDo(() ->
  //       {
  //         stopHoodOnce();
  //         setDefaultStatorCurrentLimit();
  //         io.resetHoodTimer();
  //       }
  //     ),
  //     // Condition
  //     io.isHoodWithinZeroTolerance());
  // }

  public Command runHoodToZero() {
    return run(
            () -> {
              if (io.isHoodAtTrueZero()) {
                outputs.mode = HoodMode.SPEED;
                outputs.desiredHoodSpeed = 0.0;
                hasInitiallyBeenZeroed = true;
                setDefaultStatorCurrentLimit();
                io.resetHoodTimer();
                io.zeroHood();
                return;
              }

              if (hasInitiallyBeenZeroed
                  && inputs.hoodAngle
                      > HoodConstants.HOOD_ZEROING_APPROACH_ANGLE
                          + HoodConstants.HOOD_ZEROING_APPROACH_TOLERANCE) {
                setDefaultStatorCurrentLimit();
                outputs.mode = HoodMode.POSITION;
                outputs.desiredHoodAngle = HoodConstants.HOOD_ZEROING_APPROACH_ANGLE;
                outputs.desiredHoodSpeed = 0.0;
                return;
              }

              setReducedStatorCurrentLimit();
              outputs.mode = HoodMode.SPEED;
              outputs.desiredHoodAngle = HoodConstants.HOOD_ZEROING_APPROACH_ANGLE;
              outputs.desiredHoodSpeed = HoodConstants.HOOD_ZEROING_SPEED;
            })
        .finallyDo(
            () -> {
              outputs.mode = HoodMode.SPEED;
              outputs.desiredHoodSpeed = 0.0;
              setDefaultStatorCurrentLimit();
              io.resetHoodTimer();
            });
  }

  public BooleanSupplier isHoodAtAngle() {
    return () -> {
      boolean atAngle =
          Math.abs(outputs.desiredHoodAngle - inputs.hoodAngle)
              < HoodConstants.HOOD_ANGLE_TOLERANCE;
      Logger.recordOutput(HOOD_TABLE_KEY + "atAngle", atAngle);
      return atAngle;
    };
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

  private void setHasInitiallyBeenZeroed(BooleanSupplier bool)
  {
    hasInitiallyBeenZeroed = bool.getAsBoolean();
  }

  public void periodic() {
    io.updateInputs(inputs);
    hasInitiallyBeenZeroed = hasInitiallyBeenZeroed || DriverStation.isEnabled() && io.isHoodAtTrueZero();
    Logger.processInputs("Hood", inputs);
    outputs.upKP = upKP.get();
    outputs.upKI = upKI.get();
    outputs.upKD = upKD.get();
    outputs.upKS = upKS.get();

    outputs.downKP = downKP.get();
    outputs.downKI = downKI.get();
    outputs.downKD = downKD.get();
    outputs.downKS = downKS.get();

    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Up kP", outputs.upKP);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Up kD", outputs.upKD);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Up kS", outputs.upKS);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Down kP", outputs.downKP);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Down kD", outputs.downKD);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Output Down kS", outputs.downKS);

    Logger.recordOutput(HOOD_TABLE_KEY + "Desired Angle", outputs.desiredHoodAngle);
    Logger.recordOutput(HOOD_TABLE_KEY + "Has Been Zeroed", hasInitiallyBeenZeroed);
    Logger.recordOutput(HOOD_TABLE_KEY + "Hood Mode", outputs.mode);
    Logger.recordOutput(HOOD_TABLE_KEY + "Desired Hood Speed", outputs.desiredHoodSpeed);
    SmartDashboard.putBoolean(HOOD_TABLE_KEY + "hasInitiallyBeenZeroed", hasInitiallyBeenZeroed);
    SmartDashboard.putBoolean(HOOD_TABLE_KEY + "hasReducedCurrentLimit", hasReducedCurrentLimit);
    SmartDashboard.putBoolean(HOOD_TABLE_KEY + "isHoodWithinZeroTolerance", io.isHoodWithinZeroTolerance().getAsBoolean());

    io.applyOutputs(outputs);
  }
}
