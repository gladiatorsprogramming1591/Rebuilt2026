package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  // TODO get actual motor ids
  public static final int LEFT_SHOOTER_LEADER_MOTOR_ID =
      34; // TODO: verify if shooter motors are inverted
  public static final int LEFT_SHOOTER_FOLLOWER_MOTOR_ID = 35;

  public static final int RIGHT_SHOOTER_LEADER_MOTOR_ID = 36;
  public static final int RIGHT_SHOOTER_FOLLOWER_MOTOR_ID = 37;

  public static final double SHOOTER_MOTOR_SPEED = 0.4;
  public static final double SHOOTER_MOTOR_REDUCTION = 1;
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 70;
  public static final int SHOOTER_MAX_VELOCITY = 5200;
  public static final double SHOOTER_TOLERANCE = 0;
  public static final double SHOOTER_MOTOR_INITIAL_SHOT_SPEED =
      0; // TODO: get value for first auto shot

  public static double kP = new LoggedTunableNumber("Shooter/kP", 0.65).getAsDouble();
  public static double kI = new LoggedTunableNumber("Shooter/kI", 0.0).getAsDouble();
  public static double kD = new LoggedTunableNumber("Shooter/kD", 0.0).getAsDouble();
  public static double kV = new LoggedTunableNumber("Shooter/kV", 0.125).getAsDouble();

  public static Transform3d robotToLauncher =
      new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));
}
