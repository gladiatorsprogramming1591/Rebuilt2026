package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int INTAKE_CAN_ID = 29;
  public static final int DEPLOY_CAN_ID = 30;

  public static final double INTAKE_GEAR_RATIO = 1.0;
  public static final double DEPLOY_GEAR_RATIO = 1.0;

  public static final double INTAKE_MOTOR_REDUCTION = 1.0; // placeholder until tested
  public static final double DEPLOY_MOTOR_REDUCTION = 1.0; // placeholder until tested

  public static final double INTAKE_CURRENT_LIMIT = 60.0;
  public static final double DEPLOY_CURRENT_LIMIT = 12.0;

  // % Duty-Cycle
  public static final double INTAKE_MOTOR_SPEED = 0.70;
  public static final double INTAKE_IDLE_SPEED = 0.10; // placeholder until tested
  public static final double DEPLOY_SPEED = 0.2;
  public static final double STOW_SPEED = -0.25;
  // Amps
  public static final double DEPLOY_TORQUE_CURRENT = 10.0;
  public static final double STOW_TORQUE_CURRENT = -18.0;
  public static final double INTAKE_DELAY_SECONDS = 2.0; // placeholder until tested
}
