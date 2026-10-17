// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  // TODO***: Investigate/tune these
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 30.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Slow translation during Shoot Hub only; ordinary driving/aiming uses its existing limits.
  private static final double AUTO_SHOT_POSITION_KP = 2.0;
  private static final double AUTO_SHOT_MAX_SPEED = 0.35; // Meters/sec
  private static final double AUTO_SHOT_MAX_ACCELERATION = 0.60; // Meters/sec^2
  private static final double AUTO_SHOT_POSITION_TOLERANCE = 0.03; // Meters

  private static final LoggedTunableNumber driveLaunchKp =
      new LoggedTunableNumber("DriveCommands/Launching/kP", 4.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveLaunchKd =
      new LoggedTunableNumber("DriveCommands/Launching/kD", 0.03, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveHubAimKd =
      new LoggedTunableNumber(
          "DriveCommands/Launching/HubkD", 0.5, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveYawLaunchToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Launching/YawToleranceDeg", 10.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber drivePitchLaunchToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Launching/PitchToleranceDeg", 5.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveRollLaunchToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Launching/RollToleranceDeg", 5.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveYawPassToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Passing/YawToleranceDeg", 15.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber drivePitchPassToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Passing/PitchToleranceDeg", 5.0, Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveRollPassToleranceDeg =
      new LoggedTunableNumber(
          "DriveCommands/Passing/RollToleranceDeg", 5.0, Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber lockMetersPerSecondThreshold =
      new LoggedTunableNumber(
          "DriveCommands/Launching/LockMetersPerSecThreshold",
          0.1,
          Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber lockOmegaRadsPerSecThreshold =
      new LoggedTunableNumber(
          "DriveCommands/Launching/LockOmegaRadsPerSecThreshold",
          0.15,
          Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      new LoggedTunableNumber(
          "DriveCommands/Launching/MaxPolarVelocityRadPerSec",
          0.6,
          Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveLauncherCORMinErrorDeg =
      new LoggedTunableNumber(
          "DriveCommands/Launching/DriveLauncherCORMinErrorDeg",
          15.0,
          Constants.Tuning.DRIVE_COMMANDS);
  private static final LoggedTunableNumber driveLauncherCORMaxErrorDeg =
      new LoggedTunableNumber(
          "DriveCommands/Launching/DriveLauncherCORMaxErrorDeg",
          30.0,
          Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber driveLaunchVelocityFeedforwardScalar =
      new LoggedTunableNumber(
          "DriveCommands/Launching/VelocityFeedforwardScalar", 1, Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber driveLaunchTargetJumpRejectRad =
      new LoggedTunableNumber(
          "DriveCommands/Launching/TargetJumpRejectRad", 0.75, Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber driveLaunchTargetJumpAcceptCycles =
      new LoggedTunableNumber(
          "DriveCommands/Launching/TargetJumpAcceptCycles", 10.0, Constants.Tuning.DRIVE_COMMANDS);

  private static final LoggedTunableNumber driveLaunchTargetMaxRateRadPerSec =
      new LoggedTunableNumber(
          "DriveCommands/Launching/TargetMaxRateRadPerSec", 6.0, Constants.Tuning.DRIVE_COMMANDS);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  // TODO: Add velocity angle, see 6238 DriveCommands.java
  private static Rotation2d getHubDriveAngle() {
    Rotation2d hubAngle = ShooterCalculation.getInstance().getParameters().driveAngle();

    if (Constants.Tuning.DRIVE_COMMANDS) {
      SmartDashboard.putNumber("Hub Drive Angle", hubAngle.getDegrees());
    }
    return hubAngle;
  }

  public static Command rotateToHub(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> getHubDriveAngle());
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static boolean atLaunchGoal() {
    var passing = ShooterCalculation.getInstance().getParameters().passing();
    var rotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(Timer.getTimestamp());
    boolean inPitchAndRollTolerance =
        rotation3d.isEmpty()
            || (Math.abs(rotation3d.get().getX())
                    <= Units.degreesToRadians(
                        passing
                            ? driveRollPassToleranceDeg.get()
                            : driveRollLaunchToleranceDeg.get())
                && Math.abs(rotation3d.get().getY())
                    <= Units.degreesToRadians(
                        passing
                            ? drivePitchPassToleranceDeg.get()
                            : drivePitchLaunchToleranceDeg.get()));

    return DriverStation.isEnabled()
        && Math.abs(
                RobotState.getInstance()
                    .getRotation()
                    .minus(ShooterCalculation.getInstance().getParameters().driveAngle())
                    .getRadians())
            <= Units.degreesToRadians(
                passing ? driveYawPassToleranceDeg.get() : driveYawLaunchToleranceDeg.get())
        && inPitchAndRollTolerance;
  }

  public static Command joystickDriveWhileLaunching(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return joystickDriveWhileLaunching(drive, xSupplier, ySupplier, null);
  }

  /** Aims normally while gently returning the robot center to the previous path's endpoint. */
  public static Command aimWhileSettlingAtPathEnd(Drive drive) {
    AutoShotPositionCorrection correction = new AutoShotPositionCorrection(drive);
    return joystickDriveWhileLaunching(drive, () -> 0.0, () -> 0.0, correction)
        .beforeStarting(correction::reset)
        .finallyDo(
            () -> {
              correction.finish();
              drive.stop();
            });
  }

  private static Command joystickDriveWhileLaunching(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      AutoShotPositionCorrection positionCorrection) {
    // Create command
    return Commands.run(
        () -> {
          if (positionCorrection != null && !positionCorrection.canDrive()) {
            drive.stop();
            return;
          }
          // Run PID controller
          final var parameters = ShooterCalculation.getInstance().getParameters();
          Rotation2d robotRotation = RobotState.getInstance().getRotation();
          double headingError = parameters.driveAngle().minus(robotRotation).getRadians();
          double measuredOmega =
              parameters.passing()
                  ? RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond
                  : drive.getYawVelocityRadPerSec();

          // Calculate speeds
          boolean correctPosition = positionCorrection != null;
          Translation2d fieldRelativeLinearVelocity =
              correctPosition
                  ? positionCorrection.calculate()
                  : getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      .times(DriveConstants.maxLinearSpeed);
          if (!correctPosition && AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }

          // Only limit if launching, not passing
          if (!correctPosition && !ShooterCalculation.getInstance().getParameters().passing()) {
            // Calculate max linear velocity magnitude based on the max polar velocity
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            boolean isNullVector =
                fieldRelativeLinearVelocity.getX() == 0 && fieldRelativeLinearVelocity.getY() == 0;
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(
                            isNullVector
                                ? new Rotation2d(0)
                                : fieldRelativeLinearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance =
                ShooterCalculation.getInstance().getParameters().distanceNoLookahead();
            double hubAngle =
                driveLaunchMaxPolarVelocityRadPerSec.get()
                    * ShooterCalculation.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
              double robotLookaheadDistance =
                  robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
              maxLinearVelocityMagnitude =
                  robotLookaheadDistance
                      / ShooterCalculation.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (fieldRelativeLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
              fieldRelativeLinearVelocity =
                  fieldRelativeLinearVelocity.times(
                      maxLinearVelocityMagnitude / fieldRelativeLinearVelocity.getNorm());
            }
          }

          // Track hub motion from translation, not the derivative of our own aiming correction.
          double targetVelocity =
              parameters.passing()
                  ? parameters.driveVelocity()
                  : getHubHeadingVelocity(
                      AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                          .minus(drive.getPose().getTranslation()),
                      fieldRelativeLinearVelocity);
          double omegaOutput =
              parameters.passing()
                  ? targetVelocity * driveLaunchVelocityFeedforwardScalar.get()
                      + headingError * driveLaunchKp.get()
                      + (targetVelocity - measuredOmega) * driveLaunchKd.get()
                  : calculateHubTurnOutput(headingError, targetVelocity, measuredOmega);

          // Apply chassis speeds
          double corScalar =
              correctPosition
                  ? 1.0
                  : MathUtil.clamp(
                      (Math.abs(
                                  parameters
                                      .driveAngle()
                                      .minus(RobotState.getInstance().getRotation())
                                      .getDegrees())
                              - driveLauncherCORMinErrorDeg.get())
                          / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                      0.0,
                      1.0);
          Translation2d launcherToRobot =
              ShooterConstants.robotToLauncher.getTranslation().toTranslation2d().unaryMinus();
          ChassisSpeeds fieldRelativeSpeedsWithOffset =
              parameters.passing()
                  ? GeomUtil.transformVelocity(
                      new ChassisSpeeds(
                          fieldRelativeLinearVelocity.getX(),
                          fieldRelativeLinearVelocity.getY(),
                          omegaOutput),
                      launcherToRobot.times(1.0 - corScalar),
                      robotRotation)
                  : getHubChassisSpeeds(
                      fieldRelativeLinearVelocity,
                      omegaOutput,
                      launcherToRobot.times(1.0 - corScalar),
                      robotRotation);

          // Apply X-lock
          boolean xLock =
              Math.hypot(
                          fieldRelativeSpeedsWithOffset.vxMetersPerSecond,
                          fieldRelativeSpeedsWithOffset.vyMetersPerSecond)
                      < lockMetersPerSecondThreshold.get()
                  && Math.abs(fieldRelativeSpeedsWithOffset.omegaRadiansPerSecond)
                      < lockOmegaRadsPerSecThreshold.get()
                  && (!correctPosition || positionCorrection.atPosition())
                  && (parameters.passing()
                      || (Math.abs(measuredOmega) < lockOmegaRadsPerSecThreshold.get()
                          && Math.abs(headingError) * driveLaunchKp.get()
                              < lockOmegaRadsPerSecThreshold.get()));
          if (Constants.Tuning.DRIVE_COMMANDS) {
            Logger.recordOutput("DriveCommands/Launching/XLock", xLock);
          }
          if (xLock) {
            drive.stopWithX();
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeedsWithOffset, RobotState.getInstance().getRotation()));
          }

          // Override robot setpoint speeds published by drive. We run our calculations using the
          // speeds that will ultimately be applied once we are using the full robot-to-launcher
          // transform. This prevents the setpoint from changing due to the shifting COR of the
          // robot.
          ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
              correctPosition
                  ? fieldRelativeSpeedsWithOffset
                  : GeomUtil.transformVelocity(
                      new ChassisSpeeds(
                          fieldRelativeLinearVelocity.getX(),
                          fieldRelativeLinearVelocity.getY(),
                          omegaOutput),
                      launcherToRobot,
                      RobotState.getInstance().getRotation());
          RobotState.getInstance()
              .setRobotSetpointVelocity(
                  ChassisSpeeds.discretize(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          fieldRelativeSpeedsWithFullOffset,
                          RobotState.getInstance().getRotation()),
                      Constants.loopPeriodSecs));

          // Log detailed launch aiming telemetry only while drive command tuning is enabled.
          if (Constants.Tuning.DRIVE_COMMANDS) {
            Logger.recordOutput(
                "DriveCommands/Launching/SetpointPose",
                new Pose2d(drive.getPose().getTranslation(), parameters.driveAngle()));
            Logger.recordOutput("DriveCommands/Launching/AtGoalTolerance", atLaunchGoal());
            Logger.recordOutput(
                "DriveCommands/Launching/ErrorPosition",
                parameters.driveAngle().minus(RobotState.getInstance().getRotation()));
            Logger.recordOutput(
                "DriveCommands/Launching/ErrorVelocityRadPerSec", targetVelocity - measuredOmega);
            Logger.recordOutput(
                "DriveCommands/Launching/MeasuredPosition", RobotState.getInstance().getRotation());
            Logger.recordOutput("DriveCommands/Launching/MeasuredVelocityRadPerSec", measuredOmega);
            Logger.recordOutput(
                "DriveCommands/Launching/SetpointPosition", parameters.driveAngle());
            Logger.recordOutput(
                "DriveCommands/Launching/SetpointVelocityRadPerSec", targetVelocity);
          }
        },
        drive);
  }

  /** Position correction owned by one Shoot Hub command, never by the following path. */
  private static final class AutoShotPositionCorrection {
    private final Drive drive;
    private Translation2d target = Translation2d.kZero;
    private Translation2d velocity = Translation2d.kZero;
    private double lastTimestamp;
    private double errorMeters;
    private boolean targetAvailable;

    AutoShotPositionCorrection(Drive drive) {
      this.drive = drive;
    }

    void reset() {
      var endpoint = drive.getAutoShotPosition();
      targetAvailable = DriverStation.isAutonomousEnabled() && endpoint.isPresent();
      target = endpoint.orElse(drive.getPose().getTranslation());
      velocity = Translation2d.kZero;
      lastTimestamp = Timer.getTimestamp();
      errorMeters = target.getDistance(drive.getPose().getTranslation());
      Logger.recordOutput("AutoShotPosition/Active", true);
      Logger.recordOutput("AutoShotPosition/TargetAvailable", targetAvailable);
      Logger.recordOutput("AutoShotPosition/TargetX", target.getX());
      Logger.recordOutput("AutoShotPosition/TargetY", target.getY());
    }

    boolean canDrive() {
      Pose2d pose = drive.getPose();
      return DriverStation.isAutonomousEnabled()
          && Double.isFinite(pose.getTranslation().getX())
          && Double.isFinite(pose.getTranslation().getY())
          && Double.isFinite(pose.getRotation().getRadians())
          && Double.isFinite(target.getX())
          && Double.isFinite(target.getY());
    }

    Translation2d calculate() {
      double now = Timer.getTimestamp();
      double dt = MathUtil.clamp(now - lastTimestamp, 0.0, 0.10);
      lastTimestamp = now;
      Translation2d error = target.minus(drive.getPose().getTranslation());
      errorMeters = error.getNorm();
      if (!DriverStation.isAutonomousEnabled()
          || !targetAvailable
          || !Double.isFinite(errorMeters)) {
        velocity = Translation2d.kZero;
        return velocity;
      }

      Translation2d requested =
          errorMeters <= AUTO_SHOT_POSITION_TOLERANCE
              ? Translation2d.kZero
              : error.times(AUTO_SHOT_POSITION_KP);
      double requestedSpeed = requested.getNorm();
      if (requestedSpeed > AUTO_SHOT_MAX_SPEED) {
        requested = requested.times(AUTO_SHOT_MAX_SPEED / requestedSpeed);
      }
      Translation2d change = requested.minus(velocity);
      double maximumChange = AUTO_SHOT_MAX_ACCELERATION * dt;
      if (change.getNorm() > maximumChange) {
        change = change.times(maximumChange / change.getNorm());
      }
      velocity = velocity.plus(change);
      Logger.recordOutput("AutoShotPosition/ErrorMeters", errorMeters);
      Logger.recordOutput("AutoShotPosition/VxMetersPerSecond", velocity.getX());
      Logger.recordOutput("AutoShotPosition/VyMetersPerSecond", velocity.getY());
      Logger.recordOutput("AutoShotPosition/AtPosition", atPosition());
      return velocity;
    }

    boolean atPosition() {
      return !targetAvailable
          || (errorMeters <= AUTO_SHOT_POSITION_TOLERANCE && velocity.getNorm() < 1e-6);
    }

    void finish() {
      Logger.recordOutput("AutoShotPosition/Active", false);
      Logger.recordOutput(
          "AutoShotPosition/FinalErrorMeters",
          target.getDistance(drive.getPose().getTranslation()));
      velocity = Translation2d.kZero;
    }
  }

  /**
   * Hub heading rate from intended field translation; stationary aiming has no rate feedforward.
   */
  private static double getHubHeadingVelocity(
      Translation2d robotToHub, Translation2d fieldVelocity) {
    double distanceSquared =
        robotToHub.getX() * robotToHub.getX() + robotToHub.getY() * robotToHub.getY();
    double lateralOffset = ShooterConstants.robotToLauncher.getY();
    if (distanceSquared <= lateralOffset * lateralOffset + 1e-6) {
      return 0.0;
    }
    double tangential =
        robotToHub.getY() * fieldVelocity.getX() - robotToHub.getX() * fieldVelocity.getY();
    double radial =
        robotToHub.getX() * fieldVelocity.getX() + robotToHub.getY() * fieldVelocity.getY();
    return (tangential
            + lateralOffset * radial / Math.sqrt(distanceSquared - lateralOffset * lateralOffset))
        / distanceSquared;
  }

  /** Calculates hub turn speed from heading error and measured yaw-rate damping. */
  private static double calculateHubTurnOutput(
      double headingError, double targetVelocity, double measuredOmega) {
    return headingError * driveLaunchKp.get()
        + targetVelocity * driveLaunchVelocityFeedforwardScalar.get()
        + (targetVelocity - measuredOmega) * driveHubAimKd.get();
  }

  /** Converts launcher-centered rotation to chassis speeds using the field-rotated offset. */
  private static ChassisSpeeds getHubChassisSpeeds(
      Translation2d fieldVelocity,
      double omega,
      Translation2d launcherToRobot,
      Rotation2d robotRotation) {
    Translation2d offset = launcherToRobot.rotateBy(robotRotation);
    return new ChassisSpeeds(
        fieldVelocity.getX() - omega * offset.getY(),
        fieldVelocity.getY() + omega * offset.getX(),
        omega);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
