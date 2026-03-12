package frc.robot.subsystems.hood;

public final class HoodConstants {
  public static final int HOOD_CAN_ID = 50;
  public static final int HOOD_DIO_PORT = 0;

  // public static final double ROLLER_MAX_SPEED = 1.0;
  public static final double HOOD_CURRENT_LIMIT = 10.0; // placeholder until tested
  public static final double HOOD_ZEROING_CURRENT_LIMIT = 10.0; // placeholder until tested

  public static final double HOOD_UP_SPEED = 0.15;
  public static final double HOOD_DOWN_SPEED = -0.1;
  public static final double HOOD_MAX_SPEED = 0.5;
  public static final double HOOD_ZEROING_SPEED = -0.15;

  public static final double HOOD_ZEROING_VEL_TOLERANCE = 0.10; // in rotations
  public static final double MIN_STATIONARY_DURATION = 2.0; // in seconds

  // TODO: Find these limits. Find how to always update hood angle (even while disabled)
  public static final double HOOD_LOWER_LIMIT = 0;
  public static final double HOOD_UPPER_LIMIT = 0;

  public static final double HOOD_MOTOR_REDUCTION = (16.0 / 34) * (18.0 / 44) * (10.0 / 181);

  public static final double kS = 2.0;
  public static final double kV = 0.0;
  public static final double kP = 2.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}
