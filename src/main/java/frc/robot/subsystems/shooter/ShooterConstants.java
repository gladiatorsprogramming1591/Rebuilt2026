package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableNumber;

// spotless:off
public class ShooterConstants {
  public static final int LEFT_SHOOTER_LEADER_MOTOR_ID = 34;
  public static final int LEFT_SHOOTER_FOLLOWER_MOTOR_ID = 35;

  public static final int RIGHT_SHOOTER_LEADER_MOTOR_ID = 36;
  public static final int RIGHT_SHOOTER_FOLLOWER_MOTOR_ID = 37;

  public static final double SHOOTER_MOTOR_REDUCTION = 1.0; // Currently only used in Sim. TODO: Find and assign reduction to SensorToMechanismRatio
  public static final double SHOOTER_MOTOR_CURRENT_LIMIT = 70;
  public static final double MAX_FLYWHEEL_RPM = 5200.0; // Unused. TODO: Find max RPM at 12 V
  public static final double FLYWHEEL_TOLERANCE_RPS = 1.0; // +/- 60 RPM
  public static final double SHOOTER_AT_SPEED_TIMEOUT = 0.5;
  public static final double MAX_FLYWHEEL_CALCULATED_RPM = 1750.0; // TODO: TEMPORARY
  public static final double MAX_FLYWHEEL_LOW_CEILING_RPM = 2000.0;
  
  // Change this to true to scale down & lower max flywheel speed when ceiling clearance is low.
  public static boolean isLowCeiling = false; // TODO: Causes shooter to stop
  public static final double FLYWHEEL_LOW_CEILING_SCALER = 2 / 3;
  
  private static final double m_ShootFixedRPM = 2000.0;
  private static final double m_CoastRPM = 750.0;

  private static final double m_P = 0.45; // An error of 1 rps results in <0.45> V output. P above 0.50 causes oscillation
  private static final double m_I = 0.0; // no output for integrated error
  private static final double m_D = 0.0; // no output for error derivative
  private static final double m_S = 0.0; // Add <> V output to overcome static friction
  private static final double m_V = 0.125; // A velocity target of 1 rps results in <0.125> V output
  private static final double m_A = 0.0; // An acceleration of 1 rps/s requires <> V output
  // Motion Magic velocity configs
  // only used when calling MM control request. e.g. new MotionMagicVelocityVoltage(<rps>)
  private static final double m_MMAcceleration = 400; // Target acceleration of <400> rps/s (<0.25> seconds to max)
  private static final double m_MMJerk = 4000; // Target jerk of <4000> rps/s/s (<0.1> seconds)

  public static final String tableKey = "Shooter/";
  public static final LoggedTunableNumber kP = new LoggedTunableNumber(tableKey + "kP", m_P);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber(tableKey + "kI", m_I);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber(tableKey + "kD", m_D);
  public static final LoggedTunableNumber kS = new LoggedTunableNumber(tableKey + "kS", m_S);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber(tableKey + "kV", m_V);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber(tableKey + "kA", m_A);
  public static final LoggedTunableNumber kMMAcceleration = new LoggedTunableNumber(tableKey + "kMMAcceleration", m_MMAcceleration);
  public static final LoggedTunableNumber kMMJerk = new LoggedTunableNumber(tableKey + "kMMJerk", m_MMJerk);
  
  public static final LoggedTunableNumber shootFixedRPM = new LoggedTunableNumber(tableKey + "Shoot Fixed RPM", m_ShootFixedRPM);
  public static final LoggedTunableNumber coastRPM = new LoggedTunableNumber(tableKey + "Coast RPM", m_CoastRPM);


  public static Transform3d robotToLauncher =
      new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));
}
// spotless:on
