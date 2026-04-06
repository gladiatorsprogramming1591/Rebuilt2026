package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeLeft = new TalonFX(IntakeConstants.INTAKE_LEFT);
  private final TalonFX intakeRight = new TalonFX(IntakeConstants.INTAKE_RIGHT);
  private final TalonFX deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY);
  private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_DEPLOY_DIO_PORT);
  private final DigitalInput bottomLimit = new DigitalInput(IntakeConstants.BOTTOM_DEPLOY_DIO_PORT);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withSlot(0);
  private final TorqueCurrentFOC torqueDutyCycleControl =
      new TorqueCurrentFOC(0.0).withDeadband(1.0);

  private final StatusSignal<Angle> deployAngle = deployMotor.getPosition();
  private final StatusSignal<Current> deploySupplyCurrent = deployMotor.getSupplyCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = deployMotor.getTorqueCurrent();
  private final StatusSignal<AngularVelocity> deployAngularVelocity = deployMotor.getVelocity();
  private final StatusSignal<Temperature> intakeLeftTemp = intakeLeft.getDeviceTemp();
  private final StatusSignal<Temperature> intakeRightTemp = intakeRight.getDeviceTemp();
  private final StatusSignal<AngularVelocity> intakeLeftAngularVelocity = intakeLeft.getVelocity();
  private final StatusSignal<AngularVelocity> intakeRightAngularVelocity = intakeRight.getVelocity();

  private static final String updateDeployConfigName = "Update Deploy Configs";
  private final double initConfigTimeout = 0.250;
  private final double tunedConfigTimeout = 0.100; // Equivalent to default timeout
  private final int initConfigMaxAttempts = 5;
  private final int tunedConfigMaxAttempts = 2;

  public IntakeIOKraken() {
    if (Constants.tuningMode)
    {
      SmartDashboard.putBoolean(updateDeployConfigName, false);
    }
    var intakeLeftConfig = new TalonFXConfiguration();
    var intakeCurrentLimits = intakeLeftConfig.CurrentLimits;
    intakeCurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT;
    intakeCurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT;
    intakeCurrentLimits.SupplyCurrentLimitEnable = true;
    intakeCurrentLimits.StatorCurrentLimitEnable = true;
    intakeLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // i.e. not inverted
    var intakeRightConfig = intakeLeftConfig.clone();
    intakeRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> intakeRight.getConfigurator().apply(intakeRightConfig, initConfigTimeout));
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> intakeLeft.getConfigurator().apply(intakeLeftConfig, initConfigTimeout));
    // PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> intakeLeftLeader.getConfigurator()
    //   .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.5)));
    // TODO: Find out why follower doesn't update with output fast enough
    // intakeRightFollower.setControl(new StrictFollower(intakeLeftLeader.getDeviceID()));

    var deployConfig = new TalonFXConfiguration();
    var deployCurrentLimits = deployConfig.CurrentLimits;
    deployCurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
    deployCurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
    deployCurrentLimits.SupplyCurrentLimitEnable = true;
    deployCurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs deploySlot0 = deployConfig.Slot0;
    deploySlot0.kP = IntakeConstants.kP;
    deploySlot0.kI = IntakeConstants.kI;
    deploySlot0.kD = IntakeConstants.kD;
    deploySlot0.kV = IntakeConstants.kFF;

    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(deployConfig, initConfigTimeout));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp,
        intakeLeftAngularVelocity,
        intakeRightAngularVelocity);

    deployMotor.optimizeBusUtilization();
    intakeLeft.optimizeBusUtilization();
    intakeRight.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp,
        intakeLeftAngularVelocity,
        intakeRightAngularVelocity);

    inputs.deploySpeed = deployAngularVelocity.getValueAsDouble();
    inputs.deployTorqueCurrentFOC = deployTorqueCurrent.getValueAsDouble();
    inputs.deploySupplyCurrent = deploySupplyCurrent.getValueAsDouble();
    inputs.intakeLeftSpeed = intakeLeftAngularVelocity.getValueAsDouble();
    inputs.intakeRightSpeed = intakeRightAngularVelocity.getValueAsDouble();
    inputs.intakeLeftTemp = intakeLeftTemp.getValueAsDouble();
    inputs.intakeRightTemp = intakeRightTemp.getValueAsDouble();
    inputs.isDeployDown =
        bottomLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    inputs.isDeployUp =
        topLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    if (inputs.isDeployUp) { // Re-zero when deploy is up
      inputs.encoderOffset = -deployAngle.getValueAsDouble();
    }
    inputs.deployPosition = deployAngle.getValueAsDouble() + inputs.encoderOffset;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (Constants.tuningMode)
    {
      tuneDeployMotorConfigs(outputs);
    }

    // intakeLeft.set(outputs.appliedIntakeSpeed);
    // intakeRight.set(outputs.appliedIntakeSpeed);
    intakeLeft.setControl(torqueDutyCycleControl.withOutput(outputs.appliedIntakeSpeed * IntakeConstants.MAX_TORQUE_DUTYCYCLE));
    intakeRight.setControl(torqueDutyCycleControl.withOutput(outputs.appliedIntakeSpeed * IntakeConstants.MAX_TORQUE_DUTYCYCLE));

    switch (RobotState.getDeployMode()) {
      case POSITION:


        deployMotor.setControl(
            positionControl
                .withPosition(outputs.desiredPosition)
                .withSlot(0)
                .withFeedForward(outputs.kFF));
        break;
      case SPEED:

        deployMotor.set(outputs.appliedDeploySpeed);
        break;
      case OFF:
        deployMotor.set(0);
        break;
      default:
        System.out.println("Intake Apply Outputs Empty Default");
        break;
    }
  }

  /**
   * Applies the latest tunable TalonFX configurations to the <b>deploy motor</b>.
   * <p>
   * Only applies the configuration when the Smartdashboard boolean {@value #updateDeployConfigName} is changed from false to true (i.e. rising edge).
   * 
   * @param outputs Intake outputs where the tunable configurations are accessable
   * @see {@link #createTunedDeployMotorConfig(frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs) createTunedDeployMotorConfig()}
   * @see frc.robot.util.LoggedTunableNumber LoggedTunableNumber
   */
  private void tuneDeployMotorConfigs(IntakeIOOutputs outputs)
  {
    if (SmartDashboard.getBoolean(updateDeployConfigName, true))
      {
      SmartDashboard.putBoolean(updateDeployConfigName, false);
      
      TalonFXConfiguration tunedConfigs = createTunedDeployMotorConfig(outputs);
      Slot0Configs slot0 = tunedConfigs.Slot0;
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(slot0, tunedConfigTimeout));
    }
  }

  /**
   * Creates a TalonFX configuration with the latest tunable settings for the <b>deploy motor</b>.
   * 
   * <ul>
   *  <li> <b>Updated Configurations:</b>
   *    <ul>
          <li> {@code Slot0Configs}: P, I, D, and FF
        </ul>
   * </ul>
   * 
   * @param outputs Intake outputs where the tunable configurations are accessable
   * @return Tuned TalonFX configuration
   * @see frc.robot.subsystems.intake.Intake#periodic Intake periodic() where tunable configs are saved into outputs
   * @see frc.robot.util.LoggedTunableNumber LoggedTunableNumber
   */
  private TalonFXConfiguration createTunedDeployMotorConfig(IntakeIOOutputs outputs)
  {
      TalonFXConfiguration configs = new TalonFXConfiguration();
      Slot0Configs slot0Configs = configs.Slot0;
      slot0Configs.kP = outputs.kP;
      slot0Configs.kI = outputs.kI;
      slot0Configs.kD = outputs.kD;
      slot0Configs.kS = outputs.kFF;
      return configs;
  }
}
