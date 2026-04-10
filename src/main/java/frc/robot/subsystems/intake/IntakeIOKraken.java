package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
  private final int stowSlot = 0;
  private final int deploySlot = 1;
  private final PositionTorqueCurrentFOC torquePositionControl =
      new PositionTorqueCurrentFOC(0.0);
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
  private double rawStowPosition = 0.0;
  private double encoderOffset = 0.0;
  private double angleWithOffset = 0.0;

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
    // Find out why follower doesn't update with output fast enough
    // TODO: Try follower config again after intermittent wire fix
    // intakeRightFollower.setControl(new StrictFollower(intakeLeftLeader.getDeviceID()));

    var deployConfig = new TalonFXConfiguration();
    var deployCurrentLimits = deployConfig.CurrentLimits;
    deployCurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
    deployCurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
    deployCurrentLimits.SupplyCurrentLimitEnable = true;
    deployCurrentLimits.StatorCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs stowSlot0 = deployConfig.Slot0;
    stowSlot0.kP = IntakeConstants.StowConfigs.kP;
    stowSlot0.kI = IntakeConstants.StowConfigs.kI;
    stowSlot0.kD = IntakeConstants.StowConfigs.kD;
    stowSlot0.kV = IntakeConstants.StowConfigs.kFF;
    
    Slot1Configs deploySlot1 = deployConfig.Slot1;
    deploySlot1.kP = IntakeConstants.DeployConfigs.kP;
    deploySlot1.kI = IntakeConstants.DeployConfigs.kI;
    deploySlot1.kD = IntakeConstants.DeployConfigs.kD;
    deploySlot1.kV = IntakeConstants.DeployConfigs.kFF;

    // If stow and deploy benefit from having different MM configs, consider using DynamicMotionMagic
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html#dynamic-motion-magic
    MotionMagicConfigs stowMMConfigs = deployConfig.MotionMagic;
    stowMMConfigs.MotionMagicAcceleration = IntakeConstants.StowConfigs.kmmAcceleration;
    stowMMConfigs.MotionMagicJerk = IntakeConstants.StowConfigs.kmmJerk;

    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(deployConfig, initConfigTimeout));

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
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
    inputs.isDeployDown = bottomLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    inputs.isDeployUp = topLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    double rawAngle = deployAngle.getValueAsDouble();
    if (inputs.isDeployUp) { // Re-zero when deploy is up
      rawStowPosition = rawAngle;
      encoderOffset = -rawAngle;
      inputs.encoderOffset = encoderOffset;
    }
    angleWithOffset = rawAngle + encoderOffset;
    inputs.deployPosition = angleWithOffset;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (Constants.tuningMode)
    {
      tuneDeployMotorConfigs(outputs);
    }

    if (outputs.appliedIntakeSpeed == IntakeConstants.INTAKE_MOTOR_SPEED)
    {
      intakeLeft.setControl(torqueDutyCycleControl.withOutput(IntakeConstants.MAX_TORQUE_DUTYCYCLE.getAsDouble()));
      intakeRight.setControl(torqueDutyCycleControl.withOutput(IntakeConstants.MAX_TORQUE_DUTYCYCLE.getAsDouble()));
    } else
    {
      intakeLeft.set(outputs.appliedIntakeSpeed);
      intakeRight.set(outputs.appliedIntakeSpeed);
    }

    switch (RobotState.getDeployMode()) {

      case DEPLOY_POSITION:
        slapToPosition(deploySlot, outputs.desiredPosition, outputs.kdeployFF);
        break;

      case STOW_POSITION:
        slapToPosition(stowSlot, outputs.desiredPosition, outputs.kstowFF);
        break;

      case BUMP_POSITION:
        if (deployAngle.getValueAsDouble() < IntakeConstants.MIDDLE) {
          slapToPosition(deploySlot, outputs.desiredPosition, outputs.kdeployFF);
        } else {
          slapToPosition(stowSlot, outputs.desiredPosition, outputs.kstowFF);
        }
        break;

      case SPEED:
        deployMotor.set(outputs.appliedDeploySpeed);
        break;

      case OFF:
        deployMotor.stopMotor();
        break;

      default:
        System.out.println("Intake Apply Outputs Empty Default");
        break;
    }
  }

  // TODO: Add deadband and/or coast when passing tip-point angle
  private void slapToPosition(int slot, double position, double kFF) {
    deployMotor.setControl(
        torquePositionControl
            .withPosition(position + rawStowPosition) // Undoes offset only during control
            .withSlot(slot)
            .withFeedForward(kFF));
  }

  @Override
  public void tuneDeployMotorConfigs(IntakeIOOutputs outputs)
  {
    if (SmartDashboard.getBoolean(updateDeployConfigName, true))
      {
      SmartDashboard.putBoolean(updateDeployConfigName, false);
      
      TalonFXConfiguration tunedConfigs = createTunedDeployMotorConfig(outputs);
      Slot0Configs slot0 = tunedConfigs.Slot0;
      Slot1Configs slot1 = tunedConfigs.Slot1;
      MotionMagicConfigs mm = tunedConfigs.MotionMagic;
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(slot0, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(slot1, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> deployMotor.getConfigurator().apply(mm, tunedConfigTimeout));
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
      Slot0Configs slot0 = configs.Slot0;
      slot0.kP = outputs.kdeployP;
      slot0.kI = outputs.kdeployI;
      slot0.kD = outputs.kdeployD;
      slot0.kS = outputs.kdeployFF;
      
      Slot1Configs slot1 = configs.Slot1;
      slot1.kP = outputs.kstowP;
      slot1.kI = outputs.kstowI;
      slot1.kD = outputs.kstowD;
      slot1.kS = outputs.kstowFF;

      MotionMagicConfigs mm = configs.MotionMagic;
      mm.MotionMagicAcceleration = outputs.kstowMMAcceleration;
      mm.MotionMagicJerk = outputs.kstowMMJerk;
      return configs;
  }
}
