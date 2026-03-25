package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeLeft = new TalonFX(IntakeConstants.INTAKE_LEFT);
  private final TalonFX intakeRight = new TalonFX(IntakeConstants.INTAKE_RIGHT);
  private final TalonFX deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY);

  public IntakeIOKraken() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PhoenixUtil.tryUntilOk(5, () -> intakeLeft.getConfigurator().apply(intakeConfig, 0.25));
    intakeRight.setControl(new Follower(intakeLeft.getDeviceID(), MotorAlignmentValue.Aligned)); 

    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deployMotor.getConfigurator().apply(deployConfig, 0.25);
  }

  @Override
  public void setDeploySpeed(double speed) {
    deployMotor.set(speed);
    SmartDashboard.putNumber("Deploy Set Speed", speed);
  }

  @Override
  public void setDeployTorqueCurrentFOC(double current) {
    TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(current);
    deployMotor.setControl(torqueCurrentRequest);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
    SmartDashboard.putNumber("Intake Speed", speed);
  }

  @Override
  public void stopDeployMotor() {
    deployMotor.stopMotor();
  }

  @Override
  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.deploySpeed = deployMotor.getVelocity().getValueAsDouble();
    inputs.deployTorqueCurrentFOC = deployMotor.getTorqueCurrent().getValueAsDouble();
    inputs.deploySupplyCurrent = deployMotor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeSpeed = intakeMotor.getVelocity().getValueAsDouble();
  }
}
