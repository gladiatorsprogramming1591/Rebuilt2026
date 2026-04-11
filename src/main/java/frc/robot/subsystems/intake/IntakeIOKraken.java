package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX rollerLeftMotor = new TalonFX(IntakeConstants.ROLLER_LEFT);
  private final TalonFX rollerRightMotor = new TalonFX(IntakeConstants.ROLLER_RIGHT);
  private final TalonFX slapdownMotor = new TalonFX(IntakeConstants.SLAPDOWN_ID);
  private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_SLAPDOWN_DIO_PORT);
  private final DigitalInput bottomLimit = new DigitalInput(IntakeConstants.BOTTOM_SLAPDOWN_DIO_PORT);
  private final Trigger zeroTrigger;
  private final Trigger deployedTrigger;
  private final int stowSlot = 0;
  private final int deploySlot = 1;
  private final PositionTorqueCurrentFOC torquePositionControl =
      new PositionTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueDutyCycleControl =
      new TorqueCurrentFOC(0.0).withDeadband(1.0);

  private final StatusSignal<Angle> deployAngle = slapdownMotor.getPosition();
  private final StatusSignal<Current> deploySupplyCurrent = slapdownMotor.getSupplyCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = slapdownMotor.getTorqueCurrent();
  private final StatusSignal<AngularVelocity> deployAngularVelocity = slapdownMotor.getVelocity();
  private final StatusSignal<Temperature> intakeLeftTemp = rollerLeftMotor.getDeviceTemp();
  private final StatusSignal<Temperature> intakeRightTemp = rollerRightMotor.getDeviceTemp();
  private final StatusSignal<AngularVelocity> intakeLeftAngularVelocity = rollerLeftMotor.getVelocity();
  private final StatusSignal<AngularVelocity> intakeRightAngularVelocity = rollerRightMotor.getVelocity();
  
  private static final String updateDeployConfigName = "Update Deploy Configs";
  private final double initConfigTimeout = 0.250;
  private final double tunedConfigTimeout = 0.100; // Equivalent to default timeout
  private final double encoderConfigTimeout = 0.100;
  private final int initConfigMaxAttempts = 5;
  private final int tunedConfigMaxAttempts = 2;
  private final int encoderConfigMaxAttempts = 2;
  private int seedZeroCount = 0;
  private int seedDeployCount = 0;
  private int tuneConfigsCreated = 0;
  private double rawStowPosition = 0.0;
  private double rawDeployPosition = 0.0;
  private double encoderOffset = 0.0;
  private double angleWithOffset = 0.0;

  public IntakeIOKraken() {
    SmartDashboard.putNumber(kintakeTableKey + "Tune configs created", 0);
    SmartDashboard.putString(kintakeTableKey + "Tune slot0 stow created", "N/A");
    SmartDashboard.putString(kintakeTableKey + "Tune slot1 deploy created", "N/A");
    SmartDashboard.putString(kintakeTableKey + "Tune MM stow created", "N/A");
    if (Constants.tuningMode)
    {
      SmartDashboard.putBoolean(kintakeTableKey + updateDeployConfigName, false);
    }
    var intakeLeftConfig = new TalonFXConfiguration();
    var intakeCurrentLimits = intakeLeftConfig.CurrentLimits;
    intakeCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
    intakeCurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
    intakeCurrentLimits.SupplyCurrentLimitEnable = true;
    intakeCurrentLimits.StatorCurrentLimitEnable = true;
    intakeLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // i.e. not inverted
    var intakeRightConfig = intakeLeftConfig.clone();
    intakeRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> rollerRightMotor.getConfigurator().apply(intakeRightConfig, initConfigTimeout));
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> rollerLeftMotor.getConfigurator().apply(intakeLeftConfig, initConfigTimeout));
    // PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> intakeLeftLeader.getConfigurator()
    //   .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.5)));
    // Find out why follower doesn't update with output fast enough
    // TODO: Try follower config again after intermittent wire fix
    // intakeRightFollower.setControl(new StrictFollower(intakeLeftLeader.getDeviceID()));

    var deployConfig = new TalonFXConfiguration();
    var deployCurrentLimits = deployConfig.CurrentLimits;
    deployCurrentLimits.SupplyCurrentLimit = IntakeConstants.SLAPDOWN_SUPPLY_CURRENT_LIMIT;
    deployCurrentLimits.StatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;
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

    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> slapdownMotor.getConfigurator().apply(deployConfig, initConfigTimeout));

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

    slapdownMotor.optimizeBusUtilization();
    rollerLeftMotor.optimizeBusUtilization();
    rollerRightMotor.optimizeBusUtilization();

    zeroTrigger = new Trigger(topLimit::get);
    deployedTrigger = new Trigger(bottomLimit::get);

    zeroTrigger.onFalse(seedSlapdownPositionToZero());
    deployedTrigger.onFalse(seedSlapdownPositionToDeploy());
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

    inputs.slapdownSpeed = deployAngularVelocity.getValueAsDouble();
    inputs.slapdownTorqueCurrentFOC = deployTorqueCurrent.getValueAsDouble();
    inputs.slapdownSupplyCurrent = deploySupplyCurrent.getValueAsDouble();
    inputs.rollerLeftSpeed = intakeLeftAngularVelocity.getValueAsDouble();
    inputs.rollerRightSpeed = intakeRightAngularVelocity.getValueAsDouble();
    inputs.rollerLeftTemp = intakeLeftTemp.getValueAsDouble();
    inputs.rollerRightTemp = intakeRightTemp.getValueAsDouble();
    inputs.isSlapdownDown = bottomLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    inputs.isSlapdownUp = topLimit.get() == false; // DIO value is true unless signal is detected/sensor in place
    // stowedTrigger.onTrue(new InstantCommand(() -> deployMotor.setPosition(IntakeConstants.UP)));
    // deployedTrigger.onTrue(new InstantCommand(() -> deployMotor.setPosition(IntakeConstants.DOWN)));
    double rawAngle = deployAngle.getValueAsDouble();
    if (inputs.isSlapdownUp) { // Re-zero when deploy is up
      rawStowPosition = rawAngle;
      encoderOffset = -rawAngle;
      inputs.encoderOffset = encoderOffset;
    } else {
      if (inputs.isSlapdownDown) {
        rawDeployPosition = rawAngle; // TODO: Use this to update offset and/or clamp position output
      }
    }
    angleWithOffset = rawAngle + encoderOffset;
    inputs.position = angleWithOffset;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (Constants.tuningMode)
    {
      tuneDeployMotorConfigs(outputs);
    }

    if (outputs.appliedRollerSpeed == IntakeConstants.ROLLER_PICKUP_SPEED)
    {
      rollerLeftMotor.setControl(torqueDutyCycleControl.withOutput(IntakeConstants.MAX_TORQUE_DUTYCYCLE.getAsDouble()));
      rollerRightMotor.setControl(torqueDutyCycleControl.withOutput(IntakeConstants.MAX_TORQUE_DUTYCYCLE.getAsDouble()));
    } else
    {
      rollerLeftMotor.set(outputs.appliedRollerSpeed);
      rollerRightMotor.set(outputs.appliedRollerSpeed);
    }

    switch (RobotState.getSlapdownMode()) {

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
        slapdownMotor.set(outputs.appliedSlapdownSpeed);
        break;

      case OFF:
        slapdownMotor.stopMotor();
        break;

      default:
        System.out.println("Intake Apply Outputs Empty Default");
        break;
    }
  }

  // TODO: Add deadband and/or coast when passing tip-point angle
  private void slapToPosition(int slot, double position, double kFF) {
    slapdownMotor.setControl(
        torquePositionControl
            .withPosition(position + rawStowPosition) // Undoes offset only during control
            .withSlot(slot)
            .withFeedForward(kFF));
  }

  private Command seedSlapdownPositionToZero() {
    return new InstantCommand(() -> {
      PhoenixUtil.tryUntilOk(encoderConfigMaxAttempts, () -> slapdownMotor.setPosition(IntakeConstants.UP));
      System.out.println("Seed Slapdown To Zero Count: " + ++seedZeroCount);
    }
    );
  }

  private Command seedSlapdownPositionToDeploy() {
    return new InstantCommand(() -> {
      PhoenixUtil.tryUntilOk(encoderConfigMaxAttempts, () -> slapdownMotor.setPosition(IntakeConstants.DOWN));
      System.out.println("Seed Slapdown To Deploy Count: " + ++seedDeployCount);
    }
    );
  }

  @Override
  public void tuneDeployMotorConfigs(IntakeIOOutputs outputs)
  {
    if (SmartDashboard.getBoolean(kintakeTableKey + updateDeployConfigName, true))
      {
      SmartDashboard.putBoolean(kintakeTableKey + updateDeployConfigName, false);
      
      TalonFXConfiguration tunedConfigs = createTunedDeployMotorConfig(outputs);
      Slot0Configs slot0 = tunedConfigs.Slot0;
      Slot1Configs slot1 = tunedConfigs.Slot1;
      MotionMagicConfigs mm = tunedConfigs.MotionMagic;
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> slapdownMotor.getConfigurator().apply(slot0, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> slapdownMotor.getConfigurator().apply(slot1, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> slapdownMotor.getConfigurator().apply(mm, tunedConfigTimeout));
    }
  }

  /**
   * Creates a TalonFX configuration with the latest tunable settings for the <b>deploy motor</b>.
   * 
   * <ul>
   *  <li> <b>Updated Configurations:</b>
   *    <ul>
          <li> {@code Slot0Configs}: Stowing P, I, D, and FF
          <li> {@code Slot1Configs}: Deploying P, I, D, and FF
          <li> {@code MotionMagicConfigs}: Stowing Acceleration and Jerk
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
      slot0.kP = outputs.kstowP;
      slot0.kI = outputs.kstowI;
      slot0.kD = outputs.kstowD;
      slot0.kS = outputs.kstowFF;
      
      Slot1Configs slot1 = configs.Slot1;
      slot1.kP = outputs.kdeployP;
      slot1.kI = outputs.kdeployI;
      slot1.kD = outputs.kdeployD;
      slot1.kS = outputs.kdeployFF;

      MotionMagicConfigs mm = configs.MotionMagic;
      mm.MotionMagicAcceleration = outputs.kstowMMAcceleration;
      mm.MotionMagicJerk = outputs.kstowMMJerk;

      SmartDashboard.putNumber(kintakeTableKey + "Tune configs created", ++tuneConfigsCreated);
      SmartDashboard.putString(kintakeTableKey + "Tune slot0 stow created", configs.Slot0.toString());
      SmartDashboard.putString(kintakeTableKey + "Tune slot1 deploy created", configs.Slot1.toString());
      SmartDashboard.putString(kintakeTableKey + "Tune MM stow created", configs.MotionMagic.toString());
      return configs;
  }
}
