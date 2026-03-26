package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int INTAKE_DEPLOY = 30; 
  public static final int INTAKE_RIGHT = 29; 
  public static final int INTAKE_LEFT = 28; 

  public static final int TOP_DEPLOY_DIO_PORT = 1;

  public static final double INTAKE_GEAR_RATIO = 1.0;
  public static final double DEPLOY_GEAR_RATIO = 1.0;

  public static final double INTAKE_MOTOR_REDUCTION = 1.0; // placeholder until tested
  public static final double DEPLOY_MOTOR_REDUCTION = 1.0; // placeholder until tested

  public static final double INTAKE_CURRENT_LIMIT = 60.0;
  public static final double DEPLOY_CURRENT_LIMIT = 12.0;

  public static final double kP = 2.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;

  public static final double UP = 0.0;
  public static final double DOWN = 1.0; //placeholder until tested

  // % Duty-Cycle
  public static final double INTAKE_MOTOR_SPEED = 0.70;
  public static final double INTAKE_IDLE_SPEED = 0.10; // placeholder until tested
  public static final double DEPLOY_SPEED = 0.4;
  public static final double STOW_SPEED = -0.25;
  // Amps
  public static final double DEPLOY_TORQUE_CURRENT = 10.0;
  public static final double STOW_TORQUE_CURRENT = -18.0;
  public static final double INTAKE_DELAY_SECONDS = 2.0; // placeholder until tested
}
