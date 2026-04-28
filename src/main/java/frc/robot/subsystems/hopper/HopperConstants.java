package frc.robot.subsystems.hopper;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Constants for the hopper subsystem. */
public final class HopperConstants {
  private HopperConstants() {}

  /** Table key used for AdvantageKit and dashboard logging. */
  public static final String HOPPER_TABLE_KEY = "Hopper/";

  /** CAN ID for the belt motor. */
  public static final int BELT_CAN_ID = 31;

  /** CAN ID for the CANrange used to detect whether the hopper is empty. */
  public static final int HOPPER_EMPTY_CANRANGE_CAN_ID = 32;

  /** Normal belt motor supply current limit in amps. */
  public static final double BELT_CURRENT_LIMIT = 60.0;

  /**
   * Lower belt motor supply current limit used while intaking.
   *
   * <p>This lets the belt gently agitate fuel without using the full belt current limit.
   */
  public static final double BELT_INTAKE_CURRENT_LIMIT = 10.0;

  /**
   * Distance threshold for the hopper empty sensor.
   *
   * <p>The hopper is considered empty when the measured distance is greater than this threshold for
   * {@link #MIN_EMPTY_DURATION}.
   */
  public static final LoggedTunableNumber HOPPER_EMPTY_DISTANCE_LIMIT =
      new LoggedTunableNumber("Hopper/Hopper Empty Distance Limit", 0.475, Constants.Tuning.HOPPER);

  /**
   * Minimum time the hopper empty sensor must report empty before the hopper is considered empty.
   */
  public static final double MIN_EMPTY_DURATION = 0.2;

  private static final LoggedNetworkNumber beltMotorSpeed =
      new LoggedNetworkNumber("/Tuning/beltMotorSpeed", 1.00);

  /**
   * Returns the current tunable belt motor speed.
   *
   * <p>This is a method instead of a cached constant so dashboard tuning updates while the robot is
   * running.
   *
   * @return current belt motor speed command
   */
  public static double getBeltMotorSpeed() {
    return beltMotorSpeed.get();
  }
}
