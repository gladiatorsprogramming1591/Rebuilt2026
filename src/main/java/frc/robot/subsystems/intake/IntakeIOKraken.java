package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_CAN_ID);
  private final TalonFX deployMotor = new TalonFX(IntakeConstants.DEPLOY_CAN_ID);

  public IntakeIOKraken() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(intakeConfig, 0.25);

    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotor.getConfigurator().apply(deployConfig, 0.25);
  }

  @Override
  public void setDeployMotorVoltage(double volts) {
    deployMotor.setVoltage(volts);
  }

  @Override
  public void setIntakeMotorVoltage(double volts) {
    deployMotor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.deployAppliedVolts = deployMotor.getMotorVoltage().getValueAsDouble();
  }
}
