package frc.robot.subsystems.hopper;

import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class HopperConstants {
  // public static final double ROLLER_MAX_SPEED = 1.0;
  public static final double BELT_CURRENT_LIMIT = 40.0;
  public static final double BELT_INTAKE_CURRENT_LIMIT = 10.0;
  public static final LoggedTunableNumber HOPPER_EMPTY_DISTANCE_LIMIT =
      new LoggedTunableNumber("Hopper/Hopper Empty Distance Limit", 0.475);
  public static final double MIN_EMPTY_DURATION = 0.2;

  public static final LoggedNetworkNumber beltMotorSpeed =
      new LoggedNetworkNumber("/Tuning/beltMotorSpeed", 1.00);
  public static final double BELT_MOTOR_SPEED = beltMotorSpeed.get();

  public static final int HOPPER_EMPTY_CANRANGE_CAN_ID = 32;
  public static final int BELT_CAN_ID = 31;
}
