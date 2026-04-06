package frc.robot.subsystems.hood;

public final class HoodConstants {
  public static final int HOOD_CAN_ID = 50;
  public static final int HOOD_DIO_PORT = 0;
  public static final boolean DIO_LIMIT_TRIPPED = false; // DIO returns true when circuit is open, and false when closed (limit sensor tripped).
  public static final double LIMIT_SENSOR_DEBOUNCE_TIME = 0.5;

  public static final double HOOD_SUPPLY_CURRENT_LIMIT = 10.0;
  public static final double HOOD_STATOR_CURRENT_LIMIT = 10.0; // TODO: Works with 10 amps, but may want to increase
  public static final double HOOD_ZEROING_SUPPLY_CURRENT_LIMIT = 7.0;
  public static final double HOOD_ZEROING_STATOR_CURRENT_LIMIT = 7.0;

  public static final double HOOD_UP_SPEED = 0.15;
  public static final double HOOD_DOWN_SPEED = -0.1;
  public static final double HOOD_MAX_SPEED = 0.5;
  public static final double HOOD_ZEROING_SPEED = -0.15;
  public static final double HOOD_SET_TIMEOUT = 0.5;

  public static final double HOOD_ANGLE_TOLERANCE = 20.0;

  public static final double HOOD_ZEROING_CURRENT_THRESHOLD = 3.0;
  public static final double MIN_STATIONARY_DURATION = 0.75; // in seconds

  public static final double HOOD_LOWER_LIMIT = 0.0;
  public static final double HOOD_UPPER_LIMIT = 1000.0; // Roughly before the last 2-3 teeth on the rack (using old V2 reduction)

  // TODO: Update this for V3
  public static final double HOOD_MOTOR_REDUCTION = (16.0 / 34) * (18.0 / 44) * (10.0 / 181);

  public static final double kS = 2.0;
  public static final double kV = 0.0;
  public static final double kP = 2.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}
