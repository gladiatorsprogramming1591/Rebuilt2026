package frc.robot.subsystems.hood;

/** Constants for the hood subsystem. */
public final class HoodConstants {
  private HoodConstants() {}

  /** Hood motor CAN ID. */
  public static final int HOOD_CAN_ID = 50;

  /** DIO port for the bottom zero limit sensor. */
  public static final int HOOD_DIO_PORT = 0;

  /**
   * Raw DIO state when the bottom limit sensor is tripped.
   *
   * <p>The current sensor wiring is active low: the DIO reads false when the sensor is tripped.
   */
  public static final boolean DIO_LIMIT_TRIPPED = false;

  /** Debounce time for the bottom zero limit sensor. */
  public static final double LIMIT_SENSOR_DEBOUNCE_TIME = 0.2;

  public static final double HOOD_SUPPLY_CURRENT_LIMIT = 20.0;
  public static final double HOOD_STATOR_CURRENT_LIMIT = 20.0;

  /** Reduced stator current limit used while slowly zeroing into the hard stop. */
  public static final double HOOD_ZEROING_STATOR_CURRENT_LIMIT = 7.5;

  public static final double HOOD_UP_SPEED = 0.15;
  public static final double HOOD_DOWN_SPEED = -0.1;
  public static final double HOOD_MAX_SPEED = 0.5;

  /** Slow open-loop output used for the final zeroing move. */
  public static final double HOOD_ZEROING_SPEED = -0.01;

  /** Maximum time to wait for the hood to reach a setpoint in shooting commands. */
  public static final double HOOD_SET_TIMEOUT = 0.5;

  /** Position tolerance in current legacy hood units. */
  public static final double HOOD_ANGLE_TOLERANCE = 20.0;

  /**
   * Current threshold used by the fallback zero detector.
   *
   * <p>The fallback should only accept zero when the hood is slow and current is elevated, which
   * indicates the hood is likely stalled against the hard stop.
   */
  public static final double HOOD_ZEROING_CURRENT_THRESHOLD = 3.0;

  /** Velocity threshold used by the fallback zero detector. */
  public static final double HOOD_ZEROING_VELOCITY_THRESHOLD = 1.0;

  /**
   * Intermediate angle used when returning to zero after the hood has already been zeroed.
   *
   * <p>The hood uses position control down to this angle, then switches to slow open-loop zeroing so
   * it does not hit the hard stop from far away.
   */
  public static final double HOOD_ZEROING_APPROACH_ANGLE = 75.0;

  /** Extra range above the approach angle before using position control toward the approach point. */
  public static final double HOOD_ZEROING_APPROACH_TOLERANCE = 10.0;

  /** Minimum time the hood must appear stalled before fallback zero is accepted. */
  public static final double MIN_STATIONARY_DURATION = 0.75;

  public static final double HOOD_LOWER_LIMIT = 0.0;

  /** Roughly before the last 2 to 3 teeth on the rack using the old V2 reduction. */
  public static final double HOOD_UPPER_LIMIT = 1000.0;

  /** TODO: Update this for V3. */
  public static final double HOOD_MOTOR_REDUCTION = (16.0 / 34) * (18.0 / 44) * (10.0 / 181);

  public static final double HOOD_UP_KS = 2.0;
  public static final double HOOD_UP_KP = 2.0;
  public static final double HOOD_UP_KI = 0.0;
  public static final double HOOD_UP_KD = 0.015;

  public static final double HOOD_DOWN_KS = 0.0;
  public static final double HOOD_DOWN_KP = 0.5;
  public static final double HOOD_DOWN_KI = 0.0;
  public static final double HOOD_DOWN_KD = 0.015;

  /** Table key used for AdvantageKit and dashboard logging. */
  public static final String HOOD_TABLE_KEY = "Hood/";
}