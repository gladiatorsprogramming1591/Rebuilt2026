package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {
  public static final int SLAPDOWN_ID = 30;
  public static final int ROLLER_RIGHT = 28;
  public static final int ROLLER_LEFT = 29;

  public static final int BOTTOM_SLAPDOWN_DIO_PORT = 2;
  public static final int TOP_SLAPDOWN_DIO_PORT = 1;

  public static final double ROLLER_MOTOR_REDUCTION = 1.0; // placeholder until tested
  public static final double SLAPDOWN_MOTOR_REDUCTION = 1.0; // placeholder until tested

  public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 80.0;
  public static final double ROLLER_STATOR_CURRENT_LIMIT = 150.0;
  public static final double SLAPDOWN_SUPPLY_CURRENT_LIMIT = 50.0;
  public static final double SLAPDOWN_STATOR_CURRENT_LIMIT = 50.0;
  public static final double PEAK_FORWARD_STATOR_CURRENT_LIMIT = 40.0; 
  public static final double PEAK_REVERSE_STATOR_CURRENT_LIMIT = -38.0; 
  public static final double SLAPDOWN_CURRENT_STOP_THRESHOLD = SLAPDOWN_SUPPLY_CURRENT_LIMIT;

  public static final int STATUS_SIGNAL_UPDATE_FREQUENCY = 50;

  public class DeployConfigs
  {
    public static final double kP = 4.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;
    public static final double kmmAcceleration = 0.0; // Not currently used, only using Stow MM values
    public static final double kmmJerk = 0.0; // Not currently used, only using Stow MM values
  }

  public class StowConfigs
  {
    public static final double kP = 3.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;
    public static final double kmmAcceleration = 0.0;
    public static final double kmmJerk = 0.0;
  }

  public class StowFullConfigs
  {
    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 10.0;
    public static final double kFF = 0.0;
    public static final double kmmAcceleration = 0.0; // Not currently used, only using Stow MM values
    public static final double kmmJerk = 0.0; // Not currently used, only using Stow MM values
  }

  // Angles
  public static final double UP = -17.08056640625;
  public static final double DOWN = 0.0;
  public static final double MIDDLE = DOWN / 2;
  public static final double BUMP = MIDDLE;
  public static final double SHOOTING_STOP = UP * 0.60;
  public static final double ROLLER_STOP_CONSTRAINT = UP * 0.60;
  // Points at which the slapdown succumbs to gravity
  public static final double TIPPING_POINT = MIDDLE;
  public static final double TIP_TOWARD_STOW = TIPPING_POINT - 2;
  public static final double TIP_TOWARD_DEPLOY = TIPPING_POINT + 2;
  public static final double MIN_ANGLE = UP;
  public static final double MAX_ANGLE = DOWN;
  
  // % Duty-Cycle
  public static final double ROLLER_PICKUP_SPEED = 0.60;
  public static final double ROLLER_REVERSE_SPEED = -0.40;
  public static final double DEPLOYING_SPEED = 0.5; // 0.4
  public static final double STOWING_SPEED = -0.25;
  // Amps
  public static final double DEPLOYING_TORQUE_CURRENT = 10.0;
  public static final double STOWING_TORQUE_CURRENT = -18.0;
  private static final double m_rollerTorqueCurrent = 60.0;  
  private static final boolean m_isTorqueMode = true;  
  
  public static final String kintakeTableKey = "Intake/";
  public static final String kdeployTableKey = kintakeTableKey + "Deploy/";
  public static final String kstowTableKey = kintakeTableKey + "Stow/";
  public static final String kstowFullTableKey = kintakeTableKey + "StowFull/";
  // Tunables
  // Intake roller
  public static final LoggedTunableNumber MAX_TORQUE_DUTYCYCLE =
      new LoggedTunableNumber(kintakeTableKey + "Intake roller torque", m_rollerTorqueCurrent);
  public static final LoggedTunableBoolean IS_TORQUE_MODE =
      new LoggedTunableBoolean(kintakeTableKey + "Use Torque Mode?", m_isTorqueMode);
  // Deploy
  public static final LoggedTunableNumber deploySpeed =
      new LoggedTunableNumber(kdeployTableKey + "DeploySpeed", DEPLOYING_SPEED);
  public static final LoggedTunableNumber peakForwardStatorCurrentLimit =
      new LoggedTunableNumber(kdeployTableKey + "peakForwardStatorCurrentLimit", PEAK_FORWARD_STATOR_CURRENT_LIMIT);
  public static final LoggedTunableNumber peakReverseStatorCurrentLimit =
      new LoggedTunableNumber(kdeployTableKey + "peakReverseStatorCurrentLimit", PEAK_REVERSE_STATOR_CURRENT_LIMIT);
  public static final LoggedTunableNumber kdeployP =
      new LoggedTunableNumber(kdeployTableKey + "kdeployP", DeployConfigs.kP);
  public static final LoggedTunableNumber kdeployI =
      new LoggedTunableNumber(kdeployTableKey + "kdeployI", DeployConfigs.kI);
  public static final LoggedTunableNumber kdeployD =
      new LoggedTunableNumber(kdeployTableKey + "kdeployD", DeployConfigs.kD);
  public static final LoggedTunableNumber kdeployG =
      new LoggedTunableNumber(kdeployTableKey + "kdeployG", DeployConfigs.kG);
  public static final LoggedTunableNumber kdeployFF =
      new LoggedTunableNumber(kdeployTableKey + "kdeployFF", DeployConfigs.kFF);
  // Stow - Empty
  public static final LoggedTunableNumber kstowP =
      new LoggedTunableNumber(kstowTableKey + "kstowP", StowConfigs.kP);
  public static final LoggedTunableNumber kstowI =
      new LoggedTunableNumber(kstowTableKey + "kstowI", StowConfigs.kI);
  public static final LoggedTunableNumber kstowD =
      new LoggedTunableNumber(kstowTableKey + "kstowD", StowConfigs.kD);
  public static final LoggedTunableNumber kstowG =
      new LoggedTunableNumber(kstowTableKey + "kstowG", StowConfigs.kG);
  public static final LoggedTunableNumber kstowFF =
      new LoggedTunableNumber(kstowTableKey + "kstowFF", StowConfigs.kFF);
  // Stow - Full
  public static final LoggedTunableNumber kstowFullP =
      new LoggedTunableNumber(kstowFullTableKey + "kstowFullP", StowFullConfigs.kP);
  public static final LoggedTunableNumber kstowFullI =
      new LoggedTunableNumber(kstowFullTableKey + "kstowFullI", StowFullConfigs.kI);
  public static final LoggedTunableNumber kstowFullD =
      new LoggedTunableNumber(kstowFullTableKey + "kstowFullD", StowFullConfigs.kD);
  public static final LoggedTunableNumber kstowFullG =
      new LoggedTunableNumber(kstowFullTableKey + "kstowFullG", StowFullConfigs.kG);
  public static final LoggedTunableNumber kstowFullFF =
      new LoggedTunableNumber(kstowFullTableKey + "kstowFullFF", StowFullConfigs.kFF);

  public static final LoggedTunableNumber kMMAcceleration =
      new LoggedTunableNumber(kstowTableKey + "kMMAcceleration", StowConfigs.kmmAcceleration);
  public static final LoggedTunableNumber kMMJerk =
      new LoggedTunableNumber(kstowTableKey + "kMMJerk", StowConfigs.kmmJerk);

}
