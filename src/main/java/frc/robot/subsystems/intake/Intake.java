package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputsAutoLogged outputs = new IntakeIOOutputsAutoLogged();

  private final LoggedTunableNumber deploySpeed =
      new LoggedTunableNumber("Intake/DeploySpeed", IntakeConstants.DEPLOY_SPEED);
  private final LoggedTunableNumber intakeSpeed =
      new LoggedTunableNumber("Intake/IntakeSpeed", IntakeConstants.INTAKE_MOTOR_SPEED);
  private final LoggedTunableNumber stowSpeed =
      new LoggedTunableNumber("Intake/StowSpeed", IntakeConstants.STOW_SPEED);
  private final LoggedTunableNumber deployCurrent =
      new LoggedTunableNumber("Intake/DeployCurrent", IntakeConstants.DEPLOY_TORQUE_CURRENT);
  private final LoggedTunableNumber intakeDelaySeconds =
      new LoggedTunableNumber("Intake/IntakeDelaySeconds", IntakeConstants.INTAKE_DELAY_SECONDS);
      
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);
  private static final LoggedTunableNumber kFF = new LoggedTunableNumber("Intake/kFF", 0.0);

  private static final double intakeMaxAngle = Units.degreesToRadians(0); //TODO set the max and min angle 
  private static final double intakeMinAngle = Units.degreesToRadians(0); //TODO set the max and min angle 

  @AutoLogOutput private double goalAngle = 0.0; 


  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    
    outputs.kP = kP.getAsDouble();
    outputs.kI = kI.getAsDouble();
    outputs.kD = kD.getAsDouble();
    outputs.kFF = kFF.getAsDouble();
  
    Logger.recordOutput("Intake/Applied Intake Speed", outputs.appliedIntakeSpeed);
    Logger.recordOutput("Intake/Applied Output Speed", outputs.appliedDeploySpeed);
    Logger.recordOutput("Intake/Applied Deploy Current", outputs.appliedDeployCurrent);
    io.applyOutputs(outputs);
  }

  public void runPosition(double positionRads) {
    goalAngle = positionRads; 
    outputs.position = MathUtil.clamp(goalAngle, intakeMinAngle, intakeMaxAngle);
  }

  // public Command deployIntake() {
  //   return runEnd(
  //       () -> {
  //         io.setDeploySpeed(IntakeConstants.DEPLOY_SPEED);
  //         outputs.appliedDeploySpeed = IntakeConstants.DEPLOY_SPEED;
  //         outputs.appliedDeployCurrent = 0;
  //       },
  //       () -> {
  //         io.stopDeployMotor();
  //         outputs.appliedDeploySpeed = 0;
  //         outputs.appliedDeployCurrent = 0;
  //       });
  // }

  public void intakeToggle() {
    
  }
    


}
