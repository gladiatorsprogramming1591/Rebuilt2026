package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.RollerModeState;
import frc.robot.RobotState.SlapdownModeState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.intake.IntakeConstants.kdeployTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowFullTableKey;
import static frc.robot.subsystems.intake.IntakeConstants.kstowTableKey;

/**
 * Name conventions:
 * <ul>
 *  <li><b>Intake:</b> Whole subsystem
 *  <li><b>Slapdown:</b> Intake pivoting arm
 *    <ul>
 *      <li> <b>Deploy:</b> Full extension
 *      <li> <b>Stow:</b> Full retraction into frame perimeter
 *    </ul>
 *  <li><b>Rollers:</b> Rotating tubes that propel fuel into hopper
 * </ul>
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();
  
  private int slapdownStoppedLoopCount = 0;
  private static final int SLAPDOWN_STOPPED_LOOP_COUNT_NEEDED = 2;
  private boolean stopSlapdownOnCurrentSpike = false;
  private boolean stopSlapdownWhenNotMoving = false;
  private boolean isSlapdownStopped = true;
  private boolean overrideRollerSpeed = false;

  @AutoLogOutput private double manualAngle = 0.0;
  @AutoLogOutput private double requestedRollerSpeed = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void slapdownToPosition(double angle) {
    manualAngle = angle;
    outputs.desiredPosition = MathUtil.clamp(manualAngle, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
  }

  // TODO: Change from runEnd to let applyOutputs check for down and stop motor. Probably need new
  // states.
  public Command deploy() {
    return runEnd(
            () -> {
              isSlapdownStopped = false;
              outputs.desiredPosition = IntakeConstants.DOWN;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.DEPLOY_POSITION);
              stopSlapdownOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedSlapdownSpeed = 0;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.OFF);
            })
        .until(() -> isSlapdownStopped);
  }

  // TODO: Change from runEnd to let applyOutputs check for up and stop motor. Probably need new states.
  public Command stow() {
    return runEnd(
            () -> {
              isSlapdownStopped = false;
              outputs.desiredPosition = IntakeConstants.UP;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.STOW_POSITION);
              stopSlapdownOnCurrentSpike = true;
            },
            () -> {
              outputs.appliedSlapdownSpeed = 0;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.OFF);
            })
        .until(() -> isSlapdownStopped);
  }

  public Command stowBump() {
    return runEnd(
            () -> {
              isSlapdownStopped = false;
              outputs.desiredPosition = IntakeConstants.BUMP;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.BUMP_POSITION);
              stopSlapdownOnCurrentSpike = false;
            },
            () -> {
              outputs.appliedSlapdownSpeed = 0;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.OFF);
            });
  }

  public Command stowWhileShooting() {
    return runEnd(
            () -> {
              isSlapdownStopped = false;
              outputs.desiredPosition = IntakeConstants.SHOOTING_STOP;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.BUMP_POSITION);
              stopSlapdownOnCurrentSpike = false;
            },
            () -> {
              outputs.appliedSlapdownSpeed = 0;
              RobotState.setSlapdownMode(RobotState.SlapdownModeState.OFF);
            });
  }

  public Command deployWithSpeed() {
    return runEnd(
        () -> {
          isSlapdownStopped = false;
          System.out.println("deployWithSpeed: " + IntakeConstants.deploySpeed.getAsDouble());
          outputs.appliedSlapdownSpeed = IntakeConstants.deploySpeed.getAsDouble();
          RobotState.setSlapdownMode(RobotState.SlapdownModeState.SPEED);
          stopSlapdownOnCurrentSpike = true;
        },
        () -> {
          System.out.println("deployWithSpeed: " + 0);
          outputs.appliedSlapdownSpeed = 0;
          RobotState.setSlapdownMode(RobotState.SlapdownModeState.OFF);
        });
  }

  public Command runRollerWithoutRequirements() {
    return new RunCommand(
            () -> {
              requestedRollerSpeed = IntakeConstants.ROLLER_PICKUP_SPEED;
            });
  }
  
  public Command runRoller() {
    return runEnd(
            () -> {
              requestedRollerSpeed = IntakeConstants.ROLLER_PICKUP_SPEED;
            },
            () -> {
              requestedRollerSpeed = 0.0;
            });
  }
  public Command overrideRollerSpeedCommand() {
    return runEnd(
            () -> {
              requestedRollerSpeed = IntakeConstants.ROLLER_PICKUP_SPEED;
              overrideRollerSpeed = true;
            },
            () -> {
              requestedRollerSpeed = 0.0;
              overrideRollerSpeed = false;
            });
  }
  public Command reverseRoller() {
    return runEnd(
            () -> {
              requestedRollerSpeed = IntakeConstants.ROLLER_REVERSE_SPEED;
            },
            () -> {
              requestedRollerSpeed = 0.0;
            });
  }

  // public Command deployAndIntake() {
  //   return deploy().alongWith(runIntake());
  // }

  public Command stopIntake() {
    return run(
        () -> {
          requestedRollerSpeed = 0;
          outputs.appliedSlapdownSpeed = 0;
          RobotState.setSlapdownMode(SlapdownModeState.OFF);
        });
  }

  public void deployStop() {
    outputs.appliedSlapdownSpeed = 0;
    RobotState.setSlapdownMode(SlapdownModeState.OFF);
    isSlapdownStopped = true;
  }

  /** Returns a command that sets intake speed to 0. Doesn't impact deploy. */
  public Command stopIntakeInstant() {
    return new InstantCommand(
        () -> {
          requestedRollerSpeed = 0;
        });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // Deploy configs
    outputs.kdeployP = IntakeConstants.kdeployP.getAsDouble();
    outputs.kdeployI = IntakeConstants.kdeployI.getAsDouble();
    outputs.kdeployD = IntakeConstants.kdeployD.getAsDouble();
    outputs.kdeployG = IntakeConstants.kdeployG.getAsDouble();
    outputs.kdeployFF = IntakeConstants.kdeployFF.getAsDouble();
    // Stow configs
    outputs.kstowP = IntakeConstants.kstowP.getAsDouble();
    outputs.kstowI = IntakeConstants.kstowI.getAsDouble();
    outputs.kstowD = IntakeConstants.kstowD.getAsDouble();
    outputs.kstowFF = IntakeConstants.kstowFF.getAsDouble();
    outputs.kstowG = IntakeConstants.kstowG.getAsDouble();
    outputs.kstowMMAcceleration = IntakeConstants.kMMAcceleration.getAsDouble();
    outputs.kstowMMJerk = IntakeConstants.kMMJerk.getAsDouble();
    // Stow full configs
    outputs.kstowFullP = IntakeConstants.kstowFullP.getAsDouble();
    outputs.kstowFullI = IntakeConstants.kstowFullI.getAsDouble();
    outputs.kstowFullD = IntakeConstants.kstowFullD.getAsDouble();
    outputs.kstowFullFF = IntakeConstants.kstowFullFF.getAsDouble();
    outputs.kstowFullG = IntakeConstants.kstowFullG.getAsDouble();

    if (inputs.position < IntakeConstants.ROLLER_STOP_CONSTRAINT && !overrideRollerSpeed) {
      outputs.appliedRollerSpeed = 0.0;
    } else {
      outputs.appliedRollerSpeed = requestedRollerSpeed;
    }

    Logger.recordOutput(kintakeTableKey + "Slapdown Mode", RobotState.getSlapdownMode().toString());
    Logger.recordOutput(kintakeTableKey + "Roller Mode", RobotState.getRollerMode().toString());
    Logger.recordOutput(kintakeTableKey + "Applied Roller Speed", outputs.appliedRollerSpeed);
    Logger.recordOutput(kintakeTableKey + "Applied Slapdown Speed", outputs.appliedSlapdownSpeed);
    Logger.recordOutput(kintakeTableKey + "Applied Slapdown Current", outputs.appliedSlapdownCurrent); // ?: Not updated
    // Deploy configs
    Logger.recordOutput(kdeployTableKey + "kdeployP", outputs.kdeployP);
    Logger.recordOutput(kdeployTableKey + "kdeployI", outputs.kdeployI);
    Logger.recordOutput(kdeployTableKey + "kdeployD", outputs.kdeployD);
    Logger.recordOutput(kdeployTableKey + "kdeployG", outputs.kdeployG);
    Logger.recordOutput(kdeployTableKey + "kdeployFF", outputs.kdeployFF);
    // Stow configs
    Logger.recordOutput(kstowTableKey + "kstowP", outputs.kstowP);
    Logger.recordOutput(kstowTableKey + "kstowI", outputs.kstowI);
    Logger.recordOutput(kstowTableKey + "kstowD", outputs.kstowD);
    Logger.recordOutput(kstowTableKey + "kstowG", outputs.kstowG);
    Logger.recordOutput(kstowTableKey + "kstowFF", outputs.kstowFF);
    Logger.recordOutput(kstowTableKey + "kstowMMAcceleration", outputs.kstowMMAcceleration);
    Logger.recordOutput(kstowTableKey + "kstowMMJerk", outputs.kstowMMJerk);
    // Stow Full configs
    Logger.recordOutput(kstowFullTableKey + "kstowFullP", outputs.kstowFullP);
    Logger.recordOutput(kstowFullTableKey + "kstowFullI", outputs.kstowFullI);
    Logger.recordOutput(kstowFullTableKey + "kstowFullD", outputs.kstowFullD);
    Logger.recordOutput(kstowFullTableKey + "kstowFullG", outputs.kstowFullG);
    Logger.recordOutput(kstowFullTableKey + "kstowFullFF", outputs.kstowFullFF);

    Logger.recordOutput(kintakeTableKey + "desiredPosition", outputs.desiredPosition);
    io.applyOutputs(outputs);

    // For testing, turn deploy motors off after hitting hard stop
    // TODO: Don't leave this in code as we will likely stop between bottom and top when hopper is full
    if (stopSlapdownWhenNotMoving) {
      if (inputs.RPS_RollerLeft == 0) {
        slapdownStoppedLoopCount++;
      } else {
        slapdownStoppedLoopCount = 0;
      }
      if (slapdownStoppedLoopCount > SLAPDOWN_STOPPED_LOOP_COUNT_NEEDED) {
        deployStop();
      }

      if (IntakeConstants.IS_TORQUE_MODE.get()) {
        RobotState.setRollerMode(RollerModeState.TORQUE_CURRENT);
      } else {
        RobotState.setRollerMode(RollerModeState.DUTYCYCLE);
      }
    }

    if (stopSlapdownOnCurrentSpike
        && inputs.slapdownSupplyCurrent >= IntakeConstants.SLAPDOWN_CURRENT_STOP_THRESHOLD) {
      deployStop();
    }

    // Stop the deploy motor if our sensors detect we are down or up
    if ((outputs.desiredPosition == IntakeConstants.DOWN) && inputs.isSlapdownDown
        || (outputs.desiredPosition == IntakeConstants.UP) && inputs.isSlapdownUp) {
      deployStop();
    }
  }
}
