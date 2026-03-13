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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOKraken;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOKraken;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AutoManager;
import frc.robot.util.HubShiftUtil;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Kicker kicker;
  private final Roller roller;
  private final Shooter shooter;
  private final Vision vision;
  private final Hood hood;

  // Controller
  private final CommandXboxController driver_controller = new CommandXboxController(0);
  private final CommandXboxController operator_controller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Start logging
    DataLogManager.start();
    // Record DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        Camera cam =
            robotInitConstants.isCompBot
                ? CameraConstants.RobotCameras.RIGHT
                : CameraConstants.RobotCameras.LEFT;
        vision = new Vision(cam);

        if (robotInitConstants.isCompBot) {
          intake = new Intake(new IntakeIOKraken());
          kicker = new Kicker(new KickerIOKraken());
          roller = new Roller(new RollerIOKraken());
          shooter = new Shooter(new ShooterIOKraken());
          hood = new Hood(new HoodIOKraken());
        } else {
          intake = new Intake(new IntakeIOSim());
          kicker = new Kicker(new KickerIOSim());
          roller = new Roller(new RollerIOSim());
          shooter = new Shooter(new ShooterIOSim());
          hood = new Hood(new HoodIOSim());
        }
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOSim());
        kicker = new Kicker(new KickerIOSim());
        roller = new Roller(new RollerIOSim());
        vision = new Vision();
        shooter = new Shooter(new ShooterIOSim());
        hood = new Hood(new HoodIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new Intake(new IntakeIO() {});
        kicker = new Kicker(new KickerIO() {});
        roller = new Roller(new RollerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision = new Vision();
        hood = new Hood(new HoodIO() {});

        break;
    }

    // Connect the gyro as the default vision yaw supplier
    vision.setYawSupplier(drive::getGyroRotation);

    registerNamedCommands();

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoManager = new AutoManager(drive);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autoManager.getChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ===================================== Driver Controls =====================================

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver_controller.getLeftY(),
            () -> -driver_controller.getLeftX(),
            () -> -driver_controller.getRightX()));

    roller.setDefaultCommand(roller.stopRollerMotors());
    kicker.setDefaultCommand(kicker.stopKickerMotor());
    intake.setDefaultCommand(intake.stopIntakeMotor());
    shooter.setDefaultCommand(shooter.runIdleCommand());
    hood.setDefaultCommand(hood.runHoodToZero());
    // drive base

    // Lock to 0° when A button is held
    driver_controller
        .a()
        .whileTrue(
            DriveCommands.rotateToHub(
                drive, () -> -driver_controller.getLeftY(), () -> -driver_controller.getLeftX()));

    // Switch to X pattern when X button is pressed
    driver_controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driver_controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // shooter
    driver_controller.y().whileTrue(shooter.runShooterTarget().alongWith(hood.runHoodTarget()));
    // intake
    driver_controller
        .leftTrigger()
        .toggleOnTrue(intake.runIntakeMotor().alongWith(roller.startRollerMotors()));
    driver_controller
        .rightBumper()
        .whileTrue(
            intake.deployAndIntake()); // TODO: needs to be a toggle eventually that run until a
    // certain position
    driver_controller
        .leftBumper()
        .whileTrue(
            intake.runStow()); // TODO: needs to be a toggle eventually that runs until a certain
    // encoder value
    // roller
    driver_controller.x().whileTrue(roller.runTopRollerMotor());
    // hood
    driver_controller.back().whileTrue(hood.runHoodTarget());
    driver_controller.povUp().whileTrue(hood.runHoodUp());
    driver_controller.povDown().whileTrue(hood.runHoodDown());
    // kicker
    driver_controller.povRight().toggleOnTrue(kicker.runKickerMotor());
    driver_controller.rightTrigger().whileTrue(shoot());

    // ===================================== Operator Controls =====================================
    // rollers
    operator_controller.leftBumper().whileTrue(roller.reverseRollerMotors());
    operator_controller.rightBumper().toggleOnTrue(roller.startRollerMotors());
    // intake
    operator_controller.povUp().whileTrue(intake.idleIntakeMotor());
    operator_controller.a().whileTrue(intakePulseCommand());
    operator_controller.povDown().whileTrue(intake.reverseIntakeMotor());
    // shooter
    operator_controller.b().toggleOnTrue(shooter.runShooterDutyCycle(0));
    operator_controller.leftTrigger().whileTrue(shootFixed());
    // hood
    driver_controller.start().whileTrue(hood.stopHood());
    operator_controller.x().onTrue(shootWithAim());
    operator_controller.y().onTrue(shootWithAimStationary());

    HubShiftUtil.setAllianceWinOverride(
        () -> {
          if (operator_controller.back().getAsBoolean()) {
            return Optional.of(true);
          }
          return Optional.empty();
        });
  }

  public Command warmUpShooterCommand() {
    return Commands.parallel(shooter.runShooterTarget(), hood.runHoodTarget());
  }

  public Command shoot() {
    return Commands.parallel(
        shooter.runShooterTarget(),
        hood.runHoodTarget(),
        Commands.sequence(
            Commands.parallel(
                Commands.waitUntil(hood.isHoodAtAngle()).withTimeout(HoodConstants.HOOD_SET_TIMEOUT),
                Commands.waitUntil(shooter.isShooterAtVelocity())
                    .withTimeout(ShooterConstants.SHOOTER_AT_SPEED_TIMEOUT)),
            Commands.parallel(
                roller.startRollerMotors(), kicker.runKickerMotor(), intakePulseCommand())));
  }

  public Command shootFixed() {
    return Commands.parallel(
        shooter.runFixedSpeedCommand(),
        hood.runHoodTarget(),
        Commands.sequence(
            Commands.parallel(
                Commands.waitUntil(hood.isHoodAtAngle()).withTimeout(HoodConstants.HOOD_SET_TIMEOUT),
                Commands.waitUntil(shooter.isShooterAtVelocity())
                    .withTimeout(ShooterConstants.SHOOTER_AT_SPEED_TIMEOUT)),
            Commands.parallel(
                roller.startRollerMotors(), kicker.runKickerMotor(), intakePulseCommand())));
  }

  public Command shootWithAim() {
    return Commands.parallel(
        shooter.runShooterTarget(),
        hood.runHoodTarget(),
        DriveCommands.rotateToHub(
                drive, () -> -driver_controller.getLeftY(), () -> -driver_controller.getLeftX())
            .withTimeout(0.5),
        Commands.sequence(
            Commands.parallel(
                Commands.waitUntil(hood.isHoodAtAngle()).withTimeout(HoodConstants.HOOD_SET_TIMEOUT),
                Commands.waitUntil(shooter.isShooterAtVelocity())
                    .withTimeout(ShooterConstants.SHOOTER_AT_SPEED_TIMEOUT)),
            Commands.parallel(
                roller.startRollerMotors(), kicker.runKickerMotor(), intakePulseCommand())));
  }

  public Command shootWithAimStationary() {
    return Commands.parallel(
        shooter.runShooterTarget(),
        hood.runHoodTarget(),
        DriveCommands.rotateToHub(drive, () -> 0, () -> 0)
            .withTimeout(0.5)
            .andThen(
                DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0).withTimeout(0.1)),
        Commands.sequence(
            Commands.parallel(
                hood.runHoodTarget().raceWith(Commands.waitSeconds(HoodConstants.HOOD_SET_TIMEOUT)),
                Commands.waitUntil(shooter.isShooterAtVelocity())
                    .withTimeout(ShooterConstants.SHOOTER_AT_SPEED_TIMEOUT)),
            Commands.parallel(
                roller.startRollerMotors(), kicker.runKickerMotor(), intakePulseCommand())));
  }

  // public Command intakeAndKickerAndRollerAndStow() { //name suggestions not welcome
  //   return intake.runIntakeMotor()
  //   .andThen(Command.WaitCommand(5))
  //   .andThen(intake.runStow());
  // }

  // TODO: Add this to shoot command
  public Command intakePulseCommand() {
    return intake
        .idleIntakeMotor()
        .alongWith(
            Commands.sequence(
                intake.runStow().withTimeout(0.5),
                intake.deployIntake().withTimeout(0.2),
                intake.runStow().withTimeout(0.5),
                intake.runStow().withTimeout(0.2),
                intake.deployIntake().withTimeout(0.5),
                intake.runStow().withTimeout(0.2),
                intake.runStow().withTimeout(0.5),
                intake.deployIntake().withTimeout(0.2),
                intake.runStow().withTimeout(0.5),
                intake.runStow().withTimeout(0.2),
                intake.runStow().withTimeout(0.5),
                intake.deployIntake().withTimeout(0.2),
                intake.runStow().withTimeout(0.5),
                intake.runStow().withTimeout(0.2)));
  }

  public Command intakeCommand() {
    return intake.deployIntake();
  }

  public Command intakeIn() {
    return Commands.parallel(intake.stow());
  }

  public Command idleIntake() {
    return intake.idleIntakeMotor();
  }

  public void registerNamedCommands() {
    // NamedCommands.registerCommand("Aim to Hub", );
    NamedCommands.registerCommand("Shoot Hub", shootWithAimStationary().withTimeout(4));
    NamedCommands.registerCommand("Prepare Intake", intake.deployAndIntake());
    NamedCommands.registerCommand("Intake", intakeCommand());
    NamedCommands.registerCommand("Intake In", intakeIn());
    NamedCommands.registerCommand("Warm Up Shooter", warmUpShooterCommand());
    NamedCommands.registerCommand("Lower Hood", hood.runHoodDown().withTimeout(1.0));
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
