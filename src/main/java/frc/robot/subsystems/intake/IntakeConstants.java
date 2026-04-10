package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {
  public static final int INTAKE_DEPLOY = 30;
  public static final int INTAKE_RIGHT = 28;
  public static final int INTAKE_LEFT = 29;

  public static final int BOTTOM_DEPLOY_DIO_PORT = 2;
  public static final int TOP_DEPLOY_DIO_PORT = 1;

  public static final double INTAKE_GEAR_RATIO = 1.0;
  public static final double DEPLOY_GEAR_RATIO = 1.0;

  public static final double INTAKE_MOTOR_REDUCTION = 1.0; // placeholder until tested
  public static final double DEPLOY_MOTOR_REDUCTION = 1.0; // placeholder until tested

  public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double INTAKE_STATOR_CURRENT_LIMIT = 120.0;
  public static final double DEPLOY_SUPPLY_CURRENT_LIMIT = 12.0;
  public static final double DEPLOY_STATOR_CURRENT_LIMIT = 20.0;
  public static final double DEPLOY_CURRENT_STOP_THRESHOLD = DEPLOY_SUPPLY_CURRENT_LIMIT;

  public static final int STATUS_SIGNAL_UPDATE_FREQUENCY = 50;

  public class DeployConfigs
  {
    public static final double kP = 1.45;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
  }

  public class StowConfigs
  {
    public static final double kP = 1.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kmmAcceleration = 0.0;
    public static final double kmmJerk = 0.0;
  }

  // Angles
  public static final double UP = 0.0;
  public static final double DOWN = 17.8818359375; // TODO: Find new DOWN angle after hard-stop cushioning
  public static final double MIDDLE = DOWN / 2;
  public static final double BUMP = MIDDLE;
  // Points at which the slapdown succumbs to gravity
  public static final double TIPPING_POINT = MIDDLE;
  public static final double TIP_TOWARD_STOW = TIPPING_POINT - 2;
  public static final double TIP_TOWARD_DEPLOY = TIPPING_POINT + 2;

  // % Duty-Cycle
  public static final double INTAKE_MOTOR_SPEED = 0.60;
  public static final double INTAKE_REVERSE_SPEED = -0.40;

  public static final double DEPLOY_SPEED = 0.4;
  public static final double STOW_SPEED = -0.25;
  // Amps
  public static final double DEPLOY_TORQUE_CURRENT = 10.0;
  public static final double STOW_TORQUE_CURRENT = -18.0;
  private static final double m_MaxTorqueDutyCycle = 60.0;

  public static final double INTAKE_DELAY_SECONDS = 2.0; // placeholder until tested

  public static final String INTAKE_TABLE_KEY = "Intake/";
  public static final LoggedTunableNumber MAX_TORQUE_DUTYCYCLE = new LoggedTunableNumber(INTAKE_TABLE_KEY + "Intake pick-up torque", m_MaxTorqueDutyCycle);
}
