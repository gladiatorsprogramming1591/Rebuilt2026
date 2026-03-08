package frc.robot.subsystems.roller;

import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class RollerConstants {
  // public static final double ROLLER_MAX_SPEED = 1.0;
  public static final double ROLLER_CURRENT_LIMIT = 40.0;

  private static final LoggedTunableNumber topRollerMotorSpeed =
      new LoggedTunableNumber("RollerTopSpeed/Tuning", 0.45);
  public static final double TOP_ROLLER_MOTOR_SPEED = topRollerMotorSpeed.get();

  public static final LoggedNetworkNumber bottomRollerMotorSpeed =
      new LoggedNetworkNumber("/Tuning/bottomRollerMotorSpeed", 1.00);
  public static final double BOTTOM_ROLLER_MOTOR_SPEED = bottomRollerMotorSpeed.get();

  public static final int ROLLER_TOP_CAN_ID = 32;
  public static final int ROLLER_BOTTOM_CAN_ID = 31;
}
