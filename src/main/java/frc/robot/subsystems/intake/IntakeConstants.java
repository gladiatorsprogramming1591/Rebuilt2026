package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;

/** Constants and tunable values for the intake subsystem. */
public final class IntakeConstants {
  private IntakeConstants() {}

  // Hardware IDs
  public static final int SLAPDOWN_ID = 30;
  public static final int ROLLER_RIGHT = 28;
  public static final int ROLLER_LEFT = 29;

  public static final int BOTTOM_SLAPDOWN_DIO_PORT = 2;
  public static final int TOP_SLAPDOWN_DIO_PORT = 1;

  /**
   * Raw DIO state when a slapdown limit sensor is tripped.
   *
   * <p>The slapdown sensors are active low, so the DIO reads false when the sensor is tripped.
   */
  public static final boolean SLAPDOWN_LIMIT_TRIPPED = false;

  // Mechanism reductions
  public static final double ROLLER_MOTOR_REDUCTION = 1.0; // TODO: placeholder until tested
  public static final double SLAPDOWN_MOTOR_REDUCTION = 1.0; // TODO: placeholder until tested

  // Current limits
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 80.0;
  public static final double ROLLER_STATOR_CURRENT_LIMIT = 150.0;

  public static final double SLAPDOWN_SUPPLY_CURRENT_LIMIT = 50.0;
  public static final double SLAPDOWN_STATOR_CURRENT_LIMIT = 50.0;

  public static final double PEAK_FORWARD_STATOR_CURRENT_LIMIT = 40.0;
  public static final double PEAK_REVERSE_STATOR_CURRENT_LIMIT = -38.0;

  /** Supply current threshold used to stop the slapdown after it hits a hard stop. */
  public static final double SLAPDOWN_CURRENT_STOP_THRESHOLD = SLAPDOWN_SUPPLY_CURRENT_LIMIT;

  public static final int STATUS_SIGNAL_UPDATE_FREQUENCY = 50;

  /** Default closed-loop constants for deploying the slapdown. */
  public static final class DeployConfigs {
    private DeployConfigs() {}

    public static final double kP = 4.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;

    /** Not currently used. Stow Motion Magic values are used instead. */
    public static final double kmmAcceleration = 0.0;

    /** Not currently used. Stow Motion Magic values are used instead. */
    public static final double kmmJerk = 0.0;
  }

  /** Default closed-loop constants for stowing the slapdown when empty. */
  public static final class StowConfigs {
    private StowConfigs() {}

    public static final double kP = 3.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;
    public static final double kmmAcceleration = 0.0;
    public static final double kmmJerk = 0.0;
  }

  /** Default closed-loop constants for stowing the slapdown when the hopper is full. */
  public static final class StowFullConfigs {
    private StowFullConfigs() {}

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;

    /** Not currently used. Stow Motion Magic values are used instead. */
    public static final double kmmAcceleration = 0.0;

    /** Not currently used. Stow Motion Magic values are used instead. */
    public static final double kmmJerk = 0.0;
  }

  // Slapdown positions. Current units are legacy deploy motor position units.
  public static final double UP = -17.08056640625;
  public static final double DOWN = 0.0;

  /** Midpoint between the stowed and deployed slapdown positions. */
  public static final double MIDDLE = (UP + DOWN) / 2.0;

  public static final double BUMP = MIDDLE;
  public static final double SHOOTING_STOP = UP * 0.60;

  /**
   * Roller safety cutoff.
   *
   * <p>Unless roller override is active, the rollers are stopped when the slapdown is above this
   * position so fuel is not pulled into a mostly stowed intake.
   */
  public static final double ROLLER_STOP_CONSTRAINT = UP * 0.60;

  // Points where the slapdown is expected to fall toward one side due to gravity.
  public static final double TIPPING_POINT = MIDDLE;
  public static final double TIP_TOWARD_STOW = TIPPING_POINT - 2.0;
  public static final double TIP_TOWARD_DEPLOY = TIPPING_POINT + 2.0;

  public static final double MIN_ANGLE = UP;
  public static final double MAX_ANGLE = DOWN;

  // Roller speeds
  public static final double ROLLER_PICKUP_SPEED = 0.60;
  public static final double ROLLER_REVERSE_SPEED = -0.40;

  // Manual slapdown speeds
  public static final double DEPLOYING_SPEED = 0.5;
  public static final double STOWING_SPEED = -0.25;

  // Manual slapdown torque current values
  public static final double DEPLOYING_TORQUE_CURRENT = 10.0;
  public static final double STOWING_TORQUE_CURRENT = -18.0;

  private static final double DEFAULT_ROLLER_TORQUE_CURRENT = 60.0;
  private static final boolean DEFAULT_IS_TORQUE_MODE = true;

  // Logging table keys
  public static final String kintakeTableKey = "Intake/";
  public static final String kdeployTableKey = kintakeTableKey + "Deploy/";
  public static final String kstowTableKey = kintakeTableKey + "Stow/";
  public static final String kstowFullTableKey = kintakeTableKey + "StowFull/";

  // Roller tunables
  public static final LoggedTunableNumber MAX_TORQUE_DUTYCYCLE =
      new LoggedTunableNumber(
          kintakeTableKey + "Intake roller torque",
          DEFAULT_ROLLER_TORQUE_CURRENT,
          Constants.Tuning.INTAKE);

  public static final LoggedTunableBoolean IS_TORQUE_MODE =
      new LoggedTunableBoolean(
          kintakeTableKey + "Use Torque Mode?",
          DEFAULT_IS_TORQUE_MODE,
          Constants.Tuning.INTAKE);

  // Deploy tunables
  public static final LoggedTunableNumber deploySpeed =
      new LoggedTunableNumber(
          kdeployTableKey + "DeploySpeed", DEPLOYING_SPEED, Constants.Tuning.INTAKE);

  public static final LoggedTunableNumber peakForwardStatorCurrentLimit =
      new LoggedTunableNumber(
          kdeployTableKey + "peakForwardStatorCurrentLimit",
          PEAK_FORWARD_STATOR_CURRENT_LIMIT,
          Constants.Tuning.INTAKE);

  public static final LoggedTunableNumber peakReverseStatorCurrentLimit =
      new LoggedTunableNumber(
          kdeployTableKey + "peakReverseStatorCurrentLimit",
          PEAK_REVERSE_STATOR_CURRENT_LIMIT,
          Constants.Tuning.INTAKE);

  public static final LoggedTunableNumber kdeployP =
      new LoggedTunableNumber(
          kdeployTableKey + "kdeployP", DeployConfigs.kP, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kdeployI =
      new LoggedTunableNumber(
          kdeployTableKey + "kdeployI", DeployConfigs.kI, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kdeployD =
      new LoggedTunableNumber(
          kdeployTableKey + "kdeployD", DeployConfigs.kD, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kdeployG =
      new LoggedTunableNumber(
          kdeployTableKey + "kdeployG", DeployConfigs.kG, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kdeployFF =
      new LoggedTunableNumber(
          kdeployTableKey + "kdeployFF", DeployConfigs.kFF, Constants.Tuning.INTAKE);

  // Stow tunables
  public static final LoggedTunableNumber kstowP =
      new LoggedTunableNumber(kstowTableKey + "kstowP", StowConfigs.kP, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowI =
      new LoggedTunableNumber(kstowTableKey + "kstowI", StowConfigs.kI, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowD =
      new LoggedTunableNumber(kstowTableKey + "kstowD", StowConfigs.kD, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowG =
      new LoggedTunableNumber(kstowTableKey + "kstowG", StowConfigs.kG, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowFF =
      new LoggedTunableNumber(kstowTableKey + "kstowFF", StowConfigs.kFF, Constants.Tuning.INTAKE);

  // Stow full tunables
  public static final LoggedTunableNumber kstowFullP =
      new LoggedTunableNumber(
          kstowFullTableKey + "kstowFullP", StowFullConfigs.kP, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowFullI =
      new LoggedTunableNumber(
          kstowFullTableKey + "kstowFullI", StowFullConfigs.kI, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowFullD =
      new LoggedTunableNumber(
          kstowFullTableKey + "kstowFullD", StowFullConfigs.kD, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowFullG =
      new LoggedTunableNumber(
          kstowFullTableKey + "kstowFullG", StowFullConfigs.kG, Constants.Tuning.INTAKE);
  public static final LoggedTunableNumber kstowFullFF =
      new LoggedTunableNumber(
          kstowFullTableKey + "kstowFullFF", StowFullConfigs.kFF, Constants.Tuning.INTAKE);

  public static final LoggedTunableNumber kMMAcceleration =
      new LoggedTunableNumber(
          kstowTableKey + "kMMAcceleration",
          StowConfigs.kmmAcceleration,
          Constants.Tuning.INTAKE);

  public static final LoggedTunableNumber kMMJerk =
      new LoggedTunableNumber(
          kstowTableKey + "kMMJerk", StowConfigs.kmmJerk, Constants.Tuning.INTAKE);

  /**
   * Tunable slow slapdown speed used while shooting.
   *
   * <p>This should move the intake inward toward stow. Keep this small so the intake curls in gently
   * instead of snapping back.
   */
  public static final LoggedTunableNumber shootingSlowStowSpeed =
      new LoggedTunableNumber(
          kintakeTableKey + "ShootingSlowStowSpeed", -0.20, Constants.Tuning.INTAKE);

  /**
   * Tunable slapdown stator current limit used during slow shooting stow.
   *
   * <p>This protects the mechanism while still allowing the slapdown to move at a constant low speed.
   */
  public static final LoggedTunableNumber shootingSlowStowStatorCurrentLimit =
      new LoggedTunableNumber(
          kintakeTableKey + "ShootingSlowStowStatorCurrentLimit", 80.0, Constants.Tuning.INTAKE);

  /**
   * Time used by the position-ramp shooting stow command.
   *
   * <p>Larger values make the intake curl inward more slowly while still using closed-loop position
   * control.
   */
  public static final LoggedTunableNumber shootingStowRampTimeSeconds =
      new LoggedTunableNumber(
          kintakeTableKey + "ShootingStowRampTimeSeconds", 1.0, Constants.Tuning.INTAKE);
}