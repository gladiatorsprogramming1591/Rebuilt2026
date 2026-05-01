package frc.robot.subsystems.kicker;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the kicker subsystem. */
public final class KickerConstants {
  private KickerConstants() {}

  /** Table key used for AdvantageKit and dashboard logging. */
  public static final String KICKER_TABLE_KEY = "Kicker/";

  /** CAN ID for the primary kicker motor. */
  public static final int KICKER_CAN_ID = 49;

  /** CAN ID for the secondary/follower kicker motor. */
  public static final int KICKER_2_CAN_ID = 48;

  /** True when the kicker has a second motor following the primary motor. */
  public static final boolean HAS_SECOND_KICKER_MOTOR = true;

  /** Kicker motor supply current limit in amps. */
  public static final double KICKER_SUPPLY_CURRENT_LIMIT = 30.0;

  /** Kicker motor duty-cycle closed-loop ramp period in seconds. */
  public static final double KICKER_DUTY_CYCLE_RAMP_PERIOD = 0.1;

  /** Status signal update frequency in hertz. */
  public static final int STATUS_SIGNAL_UPDATE_FREQUENCY = 50;

  private static final LoggedTunableNumber kickerSpeed =
      new LoggedTunableNumber(KICKER_TABLE_KEY + "Speed", -1.0, Constants.Tuning.KICKER);

  /**
   * Returns the current tunable kicker motor speed.
   *
   * <p>This is a method instead of a cached constant so dashboard tuning updates while the robot is
   * running.
   *
   * @return current kicker motor speed command
   */
  public static double getKickerMotorSpeed() {
    return kickerSpeed.getAsDouble();
  }
}
