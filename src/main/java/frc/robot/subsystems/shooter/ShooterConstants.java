package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;

/** Constants and tunable values for the shooter subsystem. */
public final class ShooterConstants {
  private ShooterConstants() {}

  // Hardware IDs
  public static final int LEFT_SHOOTER_LEADER_MOTOR_ID = 34;
  public static final int LEFT_SHOOTER_FOLLOWER_MOTOR_ID = 35;
  public static final int RIGHT_SHOOTER_LEADER_MOTOR_ID = 36;
  public static final int RIGHT_SHOOTER_FOLLOWER_MOTOR_ID = 37;

  // Motor configuration
  public static final double SHOOTER_MOTOR_REDUCTION = 1.0; // TODO: verify CTRE feedback ratio
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 40.0;
  public static final int STATUS_SIGNAL_UPDATE_FREQUENCY = 50;

  // Velocity limits and tolerances
  public static final double MAX_FLYWHEEL_RPM = 5200.0; // TODO: verify max RPM at 12 V
  public static final double MAX_FLYWHEEL_CALCULATED_RPM = 4000.0;
  public static final double MAX_FLYWHEEL_LOW_CEILING_RPM = 2000.0;

  /** Allowed flywheel speed error when checking whether the shooter is ready. */
  public static final double FLYWHEEL_TOLERANCE_RPM = 60.0;

  /**
   * Above idle by this much, the default command lets the shooter coast down instead of forcing
   * closed-loop deceleration.
   */
  public static final double IDLE_COAST_ENTER_MARGIN_RPM = 180.0;

  /** Once within this margin of idle, the default command resumes closed-loop idle control. */
  public static final double IDLE_COAST_EXIT_MARGIN_RPM = 60.0;

  /**
   * Change this to true to scale down and limit flywheel speed when ceiling clearance is low.
   *
   * <p>This is intentionally a runtime flag so it can be changed quickly during testing.
   */
  public static boolean isLowCeiling = false;

  /** Low-ceiling speed scale. Must use floating point division. */
  public static final double FLYWHEEL_LOW_CEILING_SCALER = 2.0 / 3.0;

  // Default tunable values
  private static final double DEFAULT_SHOOT_FIXED_RPM = 2000.0;
  private static final double DEFAULT_COAST_RPM = 1000.0;

  private static final double DEFAULT_KP = 0.45;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;
  private static final double DEFAULT_KS = 0.0;
  private static final double DEFAULT_KV = 0.125;
  private static final double DEFAULT_KA = 0.0;

  /** Conservative Motion Magic velocity acceleration until ramping is fully tuned. */
  private static final double DEFAULT_IDLE_MM_ACCELERATION = 200.0;

  private static final double DEFAULT_SHOOT_MM_ACCELERATION  = 4000.0;


  private static final double DEFAULT_MM_JERK = 8000.0;

  private static final boolean DEFAULT_USE_MOTION_MAGIC = true;

  /** Table key used for AdvantageKit and dashboard logging. */
  public static final String SHOOTER_TABLE_KEY = "Shooter/";

  /** SmartDashboard key used to push updated shooter motor configs while tuning. */
  public static final String UPDATE_CONFIG_NAME = "Update Shooter Configs";

  // Control tunables
  public static final LoggedTunableBoolean useMotionMagic =
      new LoggedTunableBoolean(
          SHOOTER_TABLE_KEY + "Use Motion Magic?",
          DEFAULT_USE_MOTION_MAGIC,
          Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kP =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kP", DEFAULT_KP, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kI =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kI", DEFAULT_KI, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kD =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kD", DEFAULT_KD, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kS =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kS", DEFAULT_KS, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kV =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kV", DEFAULT_KV, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kA =
      new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kA", DEFAULT_KA, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kMMAcceleration =
      new LoggedTunableNumber(
          SHOOTER_TABLE_KEY + "kMMAcceleration",
          DEFAULT_IDLE_MM_ACCELERATION,
          Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kMMShootAcceleration =
      new LoggedTunableNumber(
          SHOOTER_TABLE_KEY + "kMMShootAcceleration",
          DEFAULT_SHOOT_MM_ACCELERATION,
          Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber kMMJerk =
      new LoggedTunableNumber(
          SHOOTER_TABLE_KEY + "kMMJerk", DEFAULT_MM_JERK, Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber shootFixedRPM =
      new LoggedTunableNumber(
          SHOOTER_TABLE_KEY + "Shoot Fixed RPM",
          DEFAULT_SHOOT_FIXED_RPM,
          Constants.Tuning.SHOOTER);

  public static final LoggedTunableNumber coastRPM =
      new LoggedTunableNumber(
          SHOOTER_TABLE_KEY + "Coast RPM", DEFAULT_COAST_RPM, Constants.Tuning.SHOOTER);

  /** Transform from robot origin to shooter/launcher origin. */
  public static final Transform3d robotToLauncher =
      new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));
}