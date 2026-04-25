package frc.robot.subsystems.kicker;

import frc.robot.util.LoggedTunableNumber;

public final class KickerConstants {
  public static final int KICKER_CAN_ID = 49;

  // public static final double ROLLER_MAX_SPEED = 1.0;
  public static final double KICKER_CURRENT_LIMIT = 20.0; // placeholder until tested

  private static final LoggedTunableNumber kickerSpeed =
      new LoggedTunableNumber("KickerSpeed/Tuning", -.6);
  public static final double KICKER_MOTOR_SPEED = kickerSpeed.get();

  public static final int KICKER_2_CAN_ID = 48;
}
