package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.LoggedTunableBoolean;
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
  public static final double FLYWHEEL_TOLERANCE_RPS = 3.0; // +/- 60 RPM

  public static final double SHOOTER_AT_SPEED_TIMEOUT = 0.75; // 0.5. increased for now to better test "isShooterAtVelocity"
  public static final double MAX_FLYWHEEL_CALCULATED_RPM = 4000.0;
  public static final double MAX_FLYWHEEL_LOW_CEILING_RPM = 2000.0;
  
  // Change this to true to scale down & lower max flywheel speed when ceiling clearance is low.
  public static boolean isLowCeiling = false; // TODO: Causes shooter to stop
  public static final double FLYWHEEL_LOW_CEILING_SCALER = 2 / 3;
  
  private static final double m_ShootFixedRPM = 2000.0;
  private static final double m_CoastRPM = 600.0; // 750

  private static final double m_P = 0.45; // An error of 1 rps results in <0.45> V output. P above 0.50 causes oscillation
  private static final double m_I = 0.0; // no output for integrated error
  private static final double m_D = 0.0; // no output for error derivative
  private static final double m_S = 0.0; // Add <> V output to overcome static friction
  private static final double m_V = 0.125; // A velocity target of 1 rps results in <0.125> V output
  private static final double m_A = 0.0; // An acceleration of 1 rps/s requires <> V output
  // Motion Magic velocity configs
  // only used when calling MM control request. e.g. new MotionMagicVelocityVoltage(<rps>)
  // TODO: Conservative values until proper acceleration management between OFF and IDLE are in place.
  private static final double m_MMAcceleration = 200; // Target acceleration of <200> rps/s (<0.50> seconds to max)
  private static final double m_MMJerk = 2000; // Target jerk of <2000> rps/s/s (<0.2> seconds)

  private static final boolean m_useMotionMagic = true;

  /**
   * Table key (i.e. folder) for Smart Dashboard and logging.
   */
  public static final String SHOOTER_TABLE_KEY = "Shooter/";
  
  public static final LoggedTunableBoolean useMotionMagic = new LoggedTunableBoolean(SHOOTER_TABLE_KEY + "Use Motion Magic?", m_useMotionMagic);
  public static final LoggedTunableNumber kP = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kP", m_P);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kI", m_I);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kD", m_D);
  public static final LoggedTunableNumber kS = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kS", m_S);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kV", m_V);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kA", m_A);
  public static final LoggedTunableNumber kMMAcceleration = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kMMAcceleration", m_MMAcceleration);
  public static final LoggedTunableNumber kMMJerk = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "kMMJerk", m_MMJerk);
  
  public static final LoggedTunableNumber shootFixedRPM = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "Shoot Fixed RPM", m_ShootFixedRPM);
  public static final LoggedTunableNumber coastRPM = new LoggedTunableNumber(SHOOTER_TABLE_KEY + "Coast RPM", m_CoastRPM);

  // Smart Dashboard keys
    public static final String UPDATE_CONFIG_NAME = "Update Shooter Configs";

  public static Transform3d robotToLauncher =
      new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));

  public class ShooterTableKeys
  {

  }
}
// spotless:on
