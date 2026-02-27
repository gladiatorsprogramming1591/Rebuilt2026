package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int INTAKE_CAN_ID = 29;
  public static final int DEPLOY_CAN_ID = 30; // placeholder

  public static final double INTAKE_MAX_SPEED = 0.66;
  public static final double DEPLOY_MAX_SPEED = 0.33; // placeholder until tested

  public static final double INTAKE_GEAR_RATIO = 1.0;
  public static final double DEPLOY_GEAR_RATIO = 1.0;

  public static final double INTAKE_MOTOR_REDUCTION = 1.0; // placeholder until tested
  public static final double DEPLOY_MOTOR_REDUCTION = 1.0; // placeholder until tested

  public static final double INTAKE_CURRENT_LIMIT = 50.0; // placeholder until tested
  public static final double DEPLOY_CURRENT_LIMIT = 15.0; // placeholder until tested

  // % Duty-Cycle
  public static final double INTAKE_MOTOR_SPEED = 1.0; // placeholder until tested
  public static final double DEPLOY_SPEED = 1.0; // placeholder until tested
  public static final double STOW_SPEED = DEPLOY_SPEED * -1; // placeholder until tested
  // Amps
  public static final double DEPLOY_TORQUE_CURRENT = 10.0; // placeholder until tested
  public static final double STOW_TORQUE_CURRENT =
      DEPLOY_TORQUE_CURRENT * -1; // placeholder until tested
}
