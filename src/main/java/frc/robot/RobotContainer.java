// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.robotInitConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOKraken;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOKraken;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOKraken;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoManager;
import frc.robot.util.Elastic;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Declares the robot's subsystems, default commands, autonomous command chooser, and controller
 * bindings.
 *
 * <p>RobotContainer should describe how the robot is wired together. Subsystem-specific behavior
 * should stay inside subsystems, and larger multi-subsystem routines should eventually move into
 * command factory classes.
 */
public class RobotContainer {
  private final Drive drive;
  private final Intake intake;
  private final Kicker kicker;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Vision vision;
  private final Hood hood;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private double driveSpeedMultiplier = 1.0;
  private double rotationMultiplier = 1.0;

  private final LoggedTunableNumber autoStartDelay =
      new LoggedTunableNumber("Auto Start Delay", 1.0, Constants.Tuning.AUTO);

  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoManager autoManager;

  private int dashboardUpdateCounter = 0;

  /** Creates all subsystems, autonomous routines, default commands, and controller bindings. */
  public RobotContainer() {
    startLogging();

    switch (Constants.currentMode) {
      case REAL:
        drive = createRealDrive();
        vision = createRealVision();
        intake = createRealIntake();
        kicker = createRealKicker();
        hopper = createRealHopper();
        shooter = createRealShooter();
        hood = createRealHood();
        break;

      case SIM:
        drive = createSimDrive();
        vision = createSimVision();
        intake = createSimIntake();
        kicker = createSimKicker();
        hopper = createSimHopper();
        shooter = createSimShooter();
        hood = createSimHood();
        break;

      default:
        drive = createReplayDrive();
        vision = createReplayVision();
        intake = createReplayIntake();
        kicker = createReplayKicker();
        hopper = createReplayHopper();
        shooter = createReplayShooter();
        hood = createReplayHood();
        break;
    }

    vision.setYawSupplier(drive::getGyroRotation);

    registerNamedCommands();
    autoManager = new AutoManager(drive);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autoManager.getChooser());

    configureDefaultCommands();
    configureButtonBindings();
  }

  /** Starts WPILib data logging and records basic robot startup state. */
  private void startLogging() {
    DataLogManager.start();
    SmartDashboard.putBoolean("isCompBot", robotInitConstants.isCompBot);
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * Creates the real drivetrain hardware.
   *
   * @return real drivetrain subsystem
   */
  private Drive createRealDrive() {
    return new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight));
  }

  /**
   * Creates the simulated drivetrain hardware.
   *
   * @return simulated drivetrain subsystem
   */
  private Drive createSimDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(TunerConstants.FrontLeft),
        new ModuleIOSim(TunerConstants.FrontRight),
        new ModuleIOSim(TunerConstants.BackLeft),
        new ModuleIOSim(TunerConstants.BackRight));
  }

  /**
   * Creates replay drivetrain hardware with no real IO.
   *
   * @return replay drivetrain subsystem
   */
  private Drive createReplayDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {});
  }

  /**
   * Creates the real-mode vision subsystem.
   *
   * @return vision subsystem for the selected robot
   */
  private Vision createRealVision() {
    return robotInitConstants.isCompBot
        ? new Vision(RobotCameras.LEFT, RobotCameras.RIGHT)
        : new Vision(RobotCameras.PBOT_CAMERA);
  }

  /**
   * Creates the simulated vision subsystem.
   *
   * @return simulated vision subsystem
   */
  private Vision createSimVision() {
    return new Vision();
  }

  /**
   * Creates replay vision with no real cameras.
   *
   * @return replay vision subsystem
   */
  private Vision createReplayVision() {
    return new Vision();
  }

  /**
   * Creates the intake subsystem for real mode.
   *
   * <p>The comp bot uses real IO. The non-comp real configuration uses sim IO because the mechanism
   * is not present.
   *
   * @return intake subsystem
   */
  private Intake createRealIntake() {
    return robotInitConstants.isCompBot
        ? new Intake(new IntakeIOKraken())
        : new Intake(new IntakeIOSim());
  }

  /**
   * Creates the simulated intake subsystem.
   *
   * @return simulated intake subsystem
   */
  private Intake createSimIntake() {
    return new Intake(new IntakeIOSim());
  }

  /**
   * Creates replay intake with no real IO.
   *
   * @return replay intake subsystem
   */
  private Intake createReplayIntake() {
    return new Intake(new IntakeIO() {});
  }

  /**
   * Creates the kicker subsystem for real mode.
   *
   * @return kicker subsystem
   */
  private Kicker createRealKicker() {
    return robotInitConstants.isCompBot
        ? new Kicker(new KickerIOKraken())
        : new Kicker(new KickerIOSim());
  }

  /**
   * Creates the simulated kicker subsystem.
   *
   * @return simulated kicker subsystem
   */
  private Kicker createSimKicker() {
    return new Kicker(new KickerIOSim());
  }

  /**
   * Creates replay kicker with no real IO.
   *
   * @return replay kicker subsystem
   */
  private Kicker createReplayKicker() {
    return new Kicker(new KickerIO() {});
  }

  /**
   * Creates the hopper subsystem for real mode.
   *
   * @return hopper subsystem
   */
  private Hopper createRealHopper() {
    return robotInitConstants.isCompBot
        ? new Hopper(new HopperIOKraken())
        : new Hopper(new HopperIOSim());
  }

  /**
   * Creates the simulated hopper subsystem.
   *
   * @return simulated hopper subsystem
   */
  private Hopper createSimHopper() {
    return new Hopper(new HopperIOSim());
  }

  /**
   * Creates replay hopper with no real IO.
   *
   * @return replay hopper subsystem
   */
  private Hopper createReplayHopper() {
    return new Hopper(new HopperIO() {});
  }

  /**
   * Creates the shooter subsystem for real mode.
   *
   * @return shooter subsystem
   */
  private Shooter createRealShooter() {
    return robotInitConstants.isCompBot
        ? new Shooter(new ShooterIOKraken())
        : new Shooter(new ShooterIOSim());
  }

  /**
   * Creates the simulated shooter subsystem.
   *
   * @return simulated shooter subsystem
   */
  private Shooter createSimShooter() {
    return new Shooter(new ShooterIOSim());
  }

  /**
   * Creates replay shooter with no real IO.
   *
   * @return replay shooter subsystem
   */
  private Shooter createReplayShooter() {
    return new Shooter(new ShooterIO() {});
  }

  /**
   * Creates the hood subsystem for real mode.
   *
   * @return hood subsystem
   */
  private Hood createRealHood() {
    return robotInitConstants.isCompBot
        ? new Hood(new HoodIOKraken())
        : new Hood(new HoodIOSim());
  }

  /**
   * Creates the simulated hood subsystem.
   *
   * @return simulated hood subsystem
   */
  private Hood createSimHood() {
    return new Hood(new HoodIOSim());
  }

  /**
   * Creates replay hood with no real IO.
   *
   * @return replay hood subsystem
   */
  private Hood createReplayHood() {
    return new Hood(new HoodIO() {});
  }

  /** Configures subsystem default commands. */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(driverDriveCommand());
    hopper.setDefaultCommand(hopper.stopBeltMotors());
    kicker.setDefaultCommand(kicker.stopKickerMotor());
    intake.setDefaultCommand(intake.stopIntake());
    shooter.setDefaultCommand(shooterDefaultCommand());
    hood.setDefaultCommand(hood.runHoodToZero());
  }

  /**
   * Configures all driver, operator, and robot mode trigger bindings.
   *
   * <p>Bindings are split into smaller helpers so each section is easier to scan during events.
   */
  private void configureButtonBindings() {
    configureDriverControls();
    configureOperatorControls();
    configureRobotModeTriggers();
  }

  /** Configures driver controller bindings. */
  private void configureDriverControls() {
    driverController.povLeft().onTrue(setSlowModeCommand(true));
    driverController.povLeft().onFalse(setSlowModeCommand(false));

    driverController.a().whileTrue(rotateToHubCommand());
    driverController.x().onTrue(stopWithXCommand());
    driverController.b().onTrue(resetGyroCommand());
    driverController.y().whileTrue(warmUpShooterCommand());

    driverController.leftTrigger().whileTrue(prepareIntake());
    driverController.leftBumper().onTrue(intake.stow());
    driverController.rightBumper().onTrue(intake.deploy());

    // driverController.().whileTrue(hood.runHoodTarget());
    driverController.povUp().whileTrue(hood.runHoodUp());
    driverController.povDown().whileTrue(hood.runHoodDown());

    driverController.povRight().toggleOnTrue(kicker.runKickerMotor());

    Trigger shootTrigger = driverController.rightTrigger();

    shootTrigger.whileTrue(shootWithAim());
    shootTrigger.and(hopper::isHopperEmpty).whileTrue(driverRumbleCommand(0.8));

    shootTrigger
        .and(() -> !HubShiftUtil.getOfficialShiftInfo().active())
        .and(DriverStation::isTeleop)
        .onTrue(driverRumbleCommand(1.0).withTimeout(0.5));
  }

  /** Configures operator controller bindings. */
  private void configureOperatorControls() {
    operatorController.leftBumper().whileTrue(hopper.reverseBeltMotors());
    operatorController.rightBumper().toggleOnTrue(hopper.startBeltMotors());

    operatorController.povUp().whileTrue(intake.stopIntake());
    operatorController.povDown().whileTrue(intake.deploy());
    operatorController.x().whileTrue(intake.deployWithSpeed());
    operatorController.y().whileTrue(intake.stowBump());
    operatorController.b().whileTrue(intake.stow());
    operatorController.a().whileTrue(shootingIntakeStowCommand());

    operatorController.leftTrigger().whileTrue(shootFixed());

    operatorController.povLeft().whileTrue(debugHoodPositionCommand());
    operatorController.povRight().onTrue(hood.zeroHood());
    operatorController.start().debounce(1.0).onTrue(hood.runHoodToZero());

    operatorController.leftStick().whileTrue(intake.overrideRollerSpeedCommand());
    operatorController.rightStick().whileTrue(prepareIntake());
  }

  /** Configures robot mode triggers and field-management helpers. */
  private void configureRobotModeTriggers() {
    RobotModeTriggers.teleop().onTrue(initializeHubShiftCommand());
    RobotModeTriggers.autonomous().onTrue(initializeHubShiftCommand());
    RobotModeTriggers.disabled().onTrue(initializeHubShiftCommand());

    RobotModeTriggers.autonomous()
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));

    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));

    RobotModeTriggers.test()
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));

    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));

    HubShiftUtil.setAllianceWinOverride(this::getAllianceWinOverride);
  }

  /**
   * Builds the normal driver field-relative drive command.
   *
   * @return default drive command
   */
  private Command driverDriveCommand() {
    return DriveCommands.joystickDrive(
        drive,
        () -> -driverController.getLeftY() * driveSpeedMultiplier,
        () -> -driverController.getLeftX() * driveSpeedMultiplier,
        () -> -driverController.getRightX() * rotationMultiplier);
  }

  /**
   * Builds the launch-aware drive command using driver translation input.
   *
   * @return launch-aware drive command
   */
  private Command launchingDriveCommand() {
    return DriveCommands.joystickDriveWhileLaunching(
        drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX());
  }

  /**
   * Builds the launch-aware drive command with zero translation input.
   *
   * @return stationary launch-aware drive command
   */
  private Command stationaryLaunchingDriveCommand() {
    return DriveCommands.joystickDriveWhileLaunching(drive, () -> 0.0, () -> 0.0);
  }

  /**
   * Builds the rotate-to-hub command using driver translation input.
   *
   * @return rotate-to-hub command
   */
  private Command rotateToHubCommand() {
    return DriveCommands.rotateToHub(
        drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX());
  }

  /**
   * Stops the drive in an X wheel pattern.
   *
   * @return command that applies X-lock to the drivetrain
   */
  private Command stopWithXCommand() {
    return Commands.runOnce(drive::stopWithX, drive);
  }

  /**
   * Resets the robot gyro heading to zero while preserving current translation.
   *
   * @return gyro reset command
   */
  private Command resetGyroCommand() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
            drive)
        .ignoringDisable(true);
  }

  /**
   * Creates a command that changes driver slow mode.
   *
   * @param isSlow true for slow mode, false for full speed
   * @return slow mode update command
   */
  private Command setSlowModeCommand(boolean isSlow) {
    return Commands.runOnce(() -> setSlowMode(isSlow));
  }

  /**
   * Builds the shooter default command.
   *
   * <p>The timeout is intentional. It forces the conditional command to finish and re-check whether
   * the shooter should coast down or hold idle speed.
   *
   * @return shooter default command
   */
  private Command shooterDefaultCommand() {
    return shooter.coastShooterDefaultCommand();
  }

  /**
   * Builds a debug hood position command.
   *
   * @return command that moves the hood to a fixed debug position
   */
  private Command debugHoodPositionCommand() {
    return hood.runHoodPosition(() -> 500.0);
  }

  /**
   * Sets the driver translation speed mode.
   *
   * @param isSlow true to reduce driver translation speed, false for full speed
   */
  public void setSlowMode(boolean isSlow) {
    driveSpeedMultiplier = isSlow ? 0.7 : 1.0;
  }

  /**
   * Builds the shooter warmup command used before feeding fuel.
   *
   * <p>This spins the shooter, moves the hood to the target, and allows the driver to keep driving
   * with the launch-aware drive command.
   *
   * @return command that warms up shooter and hood while driving
   */
  public Command warmUpShooterCommand() {
    return Commands.parallel(shooter.runShooterTarget(), hood.runHoodTarget(), launchingDriveCommand());
  }

  /**
   * Shoots at the fixed tunable shooter speed.
   *
   * <p>This is primarily an operator/debug shooting command. It waits for the hood and shooter before
   * feeding the hopper, kicker, and slow intake stow.
   *
   * @return fixed-speed shooting command
   */
  public Command shootFixed() {
    return Commands.parallel(
        shooter.runFixedSpeedCommand(),
        hood.runHoodTarget(),
        waitUntilShooterAndHoodReady().andThen(feedShooterWithIntakeStow()));
  }

  /**
   * Shoots while aiming at the hub using driver translation input.
   *
   * <p>This is the primary teleop shooting command. It keeps updating shooter, hood, and drive aim
   * while waiting until the hood, shooter, and drive angle are ready. Once ready, it feeds fuel with
   * the hopper and kicker.
   *
   * <p>Driver intake stow/curl behavior is intentionally bound separately to right trigger plus not
   * left trigger. This lets the driver keep intaking while shooting without the intake stow command
   * fighting the intake command.
   *
   * @return aimed shooting command
   */
  public Command shootWithAim() {
  return Commands.parallel(
      shooter.runShooterTarget(),
      hood.runHoodTarget(),
      launchingDriveCommand(),
      waitUntilReadyToLaunch()
          .andThen(feedShooterWithDriverIntakeStow()));
}

  /**
   * Shoots while holding the drive translation command at zero.
   *
   * <p>This is used by autonomous named commands where the robot should aim and shoot from a fixed
   * location.
   *
   * @return stationary aimed shooting command
   */
  public Command shootWithAimStationary() {
    return Commands.parallel(
        shooter.runShooterTarget(),
        hood.runHoodTarget(),
        stationaryLaunchingDriveCommand(),
        waitUntilShooterAndHoodReady().andThen(feedShooterWithIntakeStow()));
  }

  /**
   * Waits until hood and shooter are ready.
   *
   * @return command that ends when the hood and shooter are ready
   */
  private Command waitUntilShooterAndHoodReady() {
    return Commands.parallel(
        Commands.waitUntil(hood.isHoodAtAngle()).withTimeout(HoodConstants.HOOD_SET_TIMEOUT),
        Commands.waitUntil(shooter.isShooterAtVelocity()));
  }

  /**
   * Waits until the robot is ready to launch while aiming.
   *
   * @return command that ends when hood, shooter, and launch angle are ready
   */
  private Command waitUntilReadyToLaunch() {
    return Commands.parallel(
        Commands.waitUntil(hood.isHoodAtAngle()),
        Commands.waitUntil(shooter.isShooterAtVelocity()),
        Commands.waitUntil(DriveCommands::atLaunchGoal));
  }

  /**
   * Adds a short settle delay after readiness checks before feeding.
   *
   * @return launch settle delay command
   */
  private Command launchSettleDelayCommand() {
    return Commands.waitSeconds(0.5);
  }

  /**
   * Runs the hopper and kicker to feed fuel into the shooter.
   *
   * @return fuel feed command without intake stow
   */
  private Command feedShooter() {
    return Commands.parallel(hopper.startBeltMotors(), kicker.runKickerMotor());
  }

  /**
   * Runs the hopper, kicker, and shooting intake stow together.
   *
   * <p>This is used for fixed and autonomous shooting where there is no driver intake-trigger
   * conflict to protect.
   *
   * @return fuel feed command with intake stow
   */
  private Command feedShooterWithIntakeStow() {
    return Commands.parallel(feedShooter(), shootingIntakeStowCommand());
  }

  /**
   * Runs the hopper and kicker, then curls the intake only when the driver is not actively intaking.
   *
   * <p>This keeps the intake stow/curl behavior aligned with the actual feed phase. If the driver is
   * holding intake when feeding starts, the hopper and kicker still run, but the intake waits until the
   * driver releases intake before curling inward.
   *
   * @return driver-controlled fuel feed command with gated intake stow
   */
  private Command feedShooterWithDriverIntakeStow() {
  return Commands.parallel(
      feedShooter(),
      Commands.waitUntil(() -> !driverController.leftTrigger().getAsBoolean())
          .andThen(shootingIntakeStowCommand()));
  }

  /**
   * Chooses the intake behavior used while shooting.
   *
   * <p>Currently this uses the constant-speed slow stow command. If the position-ramp version is
   * added and tested later, this helper is the only place the shooting command needs to change.
   *
   * @return intake stow command used while shooting
   */
  private Command shootingIntakeStowCommand() {
    return intake.agitateWhileShooting();
  }

  /**
   * Deploys the intake and gently runs the hopper belt while intaking.
   *
   * @return autonomous/intaking command
   */
  public Command intakeCommand() {
    return Commands.parallel(intake.deploy(), hopper.runBeltWhileIntaking());
  }

  /**
   * Deploys the intake and runs the intake rollers.
   *
   * <p>The roller command intentionally does not require the intake subsystem, which allows it to run
   * alongside the deploy command.
   *
   * @return intake preparation command
   */
  public Command prepareIntake() {
    return Commands.parallel(intake.deploy(), intake.runRollerWithPrepareUnjamWithoutRequirements());
  }

  /**
   * Stows the intake.
   *
   * @return intake stow command
   */
  public Command intakeIn() {
    return intake.stow();
  }

  /**
   * Creates a rumble command for the driver controller.
   *
   * @param intensity rumble intensity from 0.0 to 1.0
   * @return command that rumbles while scheduled
   */
  private Command driverRumbleCommand(double intensity) {
    return Commands.runEnd(
        () -> driverController.setRumble(RumbleType.kBothRumble, intensity),
        () -> driverController.setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * Creates a command that initializes hub shift tracking.
   *
   * @return hub shift initialization command
   */
  private Command initializeHubShiftCommand() {
    return Commands.runOnce(HubShiftUtil::initialize);
  }

  /**
   * Returns the optional operator override for the active alliance.
   *
   * @return optional alliance override
   */
  private Optional<Boolean> getAllianceWinOverride() {
    if (!DriverStation.isJoystickConnected(1)) {
      return Optional.empty();
    }

    if (operatorController.back().getAsBoolean()) {
      return Optional.of(true);
    }

    return Optional.empty();
  }

  /** Registers commands used by PathPlanner autonomous routines. */
  public void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot Hub", autoShootHubCommand());
    NamedCommands.registerCommand("Prepare Intake", prepareIntake());
    NamedCommands.registerCommand("Intake", intakeCommand());
    NamedCommands.registerCommand("Intake In", intakeIn());
    NamedCommands.registerCommand("Idle Intake", intake.stopIntakeInstant());
    NamedCommands.registerCommand("Stow For Bump", stowForBumpCommand());
    NamedCommands.registerCommand("Warm Up Shooter", warmUpShooterCommand());
    NamedCommands.registerCommand("Tunable Wait", tunableAutoStartDelayCommand());
    NamedCommands.registerCommand("Lower Hood And Stop Shooting", lowerHoodAndStopShootingCommand());
  }

  /**
   * Builds the autonomous hub shooting command.
   *
   * @return autonomous shoot-hub command
   */
  private Command autoShootHubCommand() {
    return shootWithAimStationary().until(hopper::isHopperEmpty).withTimeout(3.0);
  }

  /**
   * Builds the autonomous stow-for-bump command.
   *
   * @return stow-for-bump command
   */
  private Command stowForBumpCommand() {
    return intake.stopIntakeInstant().andThen(intake.stowBump());
  }

  /**
   * Builds a wait command from the current tunable auto start delay.
   *
   * <p>This is deferred so the command reads the tunable delay when the autonomous routine schedules
   * the command, not only once during robot startup.
   *
   * @return deferred tunable wait command
   */
  private Command tunableAutoStartDelayCommand() {
    return Commands.defer(() -> Commands.waitSeconds(autoStartDelay.get()), Set.of());
  }

  /**
   * Lowers the hood and stops feed mechanisms after shooting.
   *
   * @return command used by autonomous routines after shooting
   */
  private Command lowerHoodAndStopShootingCommand() {
    return Commands.parallel(hood.runHoodToZero(), hopper.stopBeltMotors(), kicker.stopKickerMotor())
        .withTimeout(1.0);
  }

  /** Updates low-rate dashboard outputs. */
  public void updateDashboardOutputs() {
    if (++dashboardUpdateCounter < 5) {
      return;
    }

    dashboardUpdateCounter = 0;

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    updateShiftDashboardOutputs();
  }

  /** Updates hub shift dashboard outputs. */
  private void updateShiftDashboardOutputs() {
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue)
            == HubShiftUtil.getFirstActiveAlliance());
  }

  /**
   * Returns the selected autonomous command.
   *
   * @return autonomous command selected on the dashboard
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}