package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

import java.util.function.BooleanSupplier;

// spotless:off
public class ShooterIOKraken implements ShooterIO {
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
  // private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<AngularVelocity> RL_RPS;
  private final StatusSignal<AngularVelocity> RF_RPS;
  private final StatusSignal<AngularVelocity> LL_RPS;
  private final StatusSignal<AngularVelocity> LF_RPS;
  private final StatusSignal<Voltage> RL_appliedVolts;
  private final StatusSignal<Voltage> RF_appliedVolts;
  private final StatusSignal<Voltage> LL_appliedVolts;
  private final StatusSignal<Voltage> LF_appliedVolts;
  private final StatusSignal<Temperature> RL_motorTemp;
  private final StatusSignal<Temperature> RF_motorTemp;
  private final StatusSignal<Temperature> LL_motorTemp;
  private final StatusSignal<Temperature> LF_motorTemp;
  private final StatusSignal<Current> RL_supplyCurrent;
  private final StatusSignal<Current> RF_supplyCurrent;
  private final StatusSignal<Current> LL_supplyCurrent;
  private final StatusSignal<Current> LF_supplyCurrent;
  private final StatusSignal<Current> RL_torqueCurrentAmps;
  private final StatusSignal<Current> RF_torqueCurrentAmps;
  private final StatusSignal<Current> LL_torqueCurrentAmps;
  private final StatusSignal<Current> LF_torqueCurrentAmps;

  private final TalonFX rightShooterLeader =
  new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
  new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);
  
  private final TalonFX leftShooterLeader =
  new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower =
  new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);
  
  private double desiredRPS = 0.0;
  private String updateConfigName = "Update Shooter Configs";
  private double initConfigTimeout = 0.250;
  private double tunedConfigTimeout = 0.100; // Equivalent to default timeout
  private int initConfigMaxAttempts = 5;
  private int tunedConfigMaxAttempts = 2; // Equivalent to default timeout

  public ShooterIOKraken() {
    if (Constants.tuningMode)
    {
      SmartDashboard.putBoolean(updateConfigName, false);
    }
    RL_RPS = rightShooterLeader.getVelocity();
    RF_RPS = rightShooterFollower.getVelocity();
    LL_RPS = leftShooterLeader.getVelocity();
    LF_RPS = leftShooterFollower.getVelocity();
    RL_appliedVolts = rightShooterLeader.getMotorVoltage();
    RF_appliedVolts = rightShooterFollower.getMotorVoltage();
    LL_appliedVolts = leftShooterLeader.getMotorVoltage();
    LF_appliedVolts = leftShooterFollower.getMotorVoltage();
    RL_motorTemp = rightShooterLeader.getDeviceTemp();
    RF_motorTemp = rightShooterFollower.getDeviceTemp();
    LL_motorTemp = leftShooterLeader.getDeviceTemp();
    LF_motorTemp = leftShooterFollower.getDeviceTemp();
    RL_supplyCurrent = rightShooterLeader.getSupplyCurrent();
    RF_supplyCurrent = rightShooterFollower.getSupplyCurrent();
    LL_supplyCurrent = leftShooterLeader.getSupplyCurrent();
    LF_supplyCurrent = leftShooterFollower.getSupplyCurrent();
    RL_torqueCurrentAmps = rightShooterLeader.getTorqueCurrent();
    RF_torqueCurrentAmps = rightShooterFollower.getTorqueCurrent();
    LL_torqueCurrentAmps = leftShooterLeader.getTorqueCurrent();
    LF_torqueCurrentAmps = leftShooterFollower.getTorqueCurrent();

    // rightShooterLeader.optimizeBusUtilization(4, 0.1);
    // rightShooterFollower.optimizeBusUtilization(4, 0.1);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted

    Slot0Configs slot0Configs = rightConfig.Slot0;
    slot0Configs.kP = ShooterConstants.kP.getAsDouble();
    slot0Configs.kI = ShooterConstants.kI.getAsDouble();
    slot0Configs.kD = ShooterConstants.kD.getAsDouble();
    slot0Configs.kS = ShooterConstants.kS.getAsDouble();
    slot0Configs.kV = ShooterConstants.kV.getAsDouble();
    slot0Configs.kA = ShooterConstants.kA.getAsDouble();

    // Motion Magic velocity configs
    // only used when calling MM control request. e.g. new MotionMagicVelocityVoltage(<rps>)
    MotionMagicConfigs motionMagicConfigs = rightConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.kMMAcceleration.getAsDouble();
    motionMagicConfigs.MotionMagicJerk = ShooterConstants.kMMJerk.getAsDouble();

    TalonFXConfiguration leftConfig = rightConfig.clone();
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // i.e. not inverted

    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> rightShooterLeader.getConfigurator().apply(rightConfig, initConfigTimeout));
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> leftShooterLeader.getConfigurator().apply(leftConfig, initConfigTimeout));
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> rightShooterFollower.getConfigurator().apply(rightConfig, initConfigTimeout));
    PhoenixUtil.tryUntilOk(initConfigMaxAttempts, () -> leftShooterFollower.getConfigurator().apply(leftConfig, initConfigTimeout));

    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterFollower.setControl(
        new Follower(leftShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }
  // spotless:on

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        RL_RPS,
        RF_RPS,
        LL_RPS,
        LF_RPS,
        RL_appliedVolts,
        RF_appliedVolts,
        LL_appliedVolts,
        LF_appliedVolts,
        RL_motorTemp,
        RF_motorTemp,
        LL_motorTemp,
        LF_motorTemp,
        RL_supplyCurrent,
        RF_supplyCurrent,
        LL_supplyCurrent,
        LF_supplyCurrent,
        RL_torqueCurrentAmps,
        RF_torqueCurrentAmps,
        LL_torqueCurrentAmps,
        LF_torqueCurrentAmps);
    inputs.RL_RPS = RL_RPS.getValueAsDouble();
    inputs.RF_RPS = RF_RPS.getValueAsDouble();
    inputs.LL_RPS = LL_RPS.getValueAsDouble();
    inputs.LF_RPS = LF_RPS.getValueAsDouble();
    inputs.RL_appliedVolts = RL_appliedVolts.getValueAsDouble();
    inputs.RF_appliedVolts = RF_appliedVolts.getValueAsDouble();
    inputs.LL_appliedVolts = LL_appliedVolts.getValueAsDouble();
    inputs.LF_appliedVolts = LF_appliedVolts.getValueAsDouble();
    inputs.RL_motorTemp = RL_motorTemp.getValueAsDouble();
    inputs.RF_motorTemp = RF_motorTemp.getValueAsDouble();
    inputs.LL_motorTemp = LL_motorTemp.getValueAsDouble();
    inputs.LF_motorTemp = LF_motorTemp.getValueAsDouble();
    inputs.RL_supplyCurrent = RL_supplyCurrent.getValueAsDouble();
    inputs.RF_supplyCurrent = RF_supplyCurrent.getValueAsDouble();
    inputs.LL_supplyCurrent = LL_supplyCurrent.getValueAsDouble();
    inputs.LF_supplyCurrent = LF_supplyCurrent.getValueAsDouble();
    inputs.RL_torqueCurrentAmps = RL_torqueCurrentAmps.getValueAsDouble();
    inputs.RF_torqueCurrentAmps = RF_torqueCurrentAmps.getValueAsDouble();
    inputs.LL_torqueCurrentAmps = LL_torqueCurrentAmps.getValueAsDouble();
    inputs.LF_torqueCurrentAmps = LF_torqueCurrentAmps.getValueAsDouble();
    inputs.shooterAtVelocity = rightShooterAtVelocity().getAsBoolean();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    if (Constants.tuningMode)
    {
      tuneMotorConfigs(outputs);
    }

    desiredRPS = outputs.desiredVelocityRPM / 60;

    SmartDashboard.putNumber("Desired RPS", desiredRPS);
    rightShooterLeader.setControl(velocityControl.withVelocity(desiredRPS));
    leftShooterLeader.setControl(velocityControl.withVelocity(desiredRPS));
  }

  @Override
  public void runShooterDutyCycle(double dutyCycle) {
    rightShooterLeader.set(dutyCycle);
    leftShooterLeader.set(dutyCycle);
  }

  /**
   * Checks if the velocity of the right leader motor is within tolerance of the target velocity.
   * <p>
   * Only works with positive (i.e. shooting) velocity targets, not negative.
   *
   * @return Boolean supplier of whether right leader motor velocity is at target
   */
  @Override
  public BooleanSupplier rightShooterAtVelocity() {

    return (() ->
        rightShooterLeader.getVelocity().getValueAsDouble()
            > desiredRPS - ShooterConstants.FLYWHEEL_TOLERANCE_RPS);
  }

  
  private void tuneMotorConfigs(ShooterIOOutputs outputs)
  {
    if (SmartDashboard.getBoolean(updateConfigName, true))
      {
      SmartDashboard.putBoolean(updateConfigName, false);
      
      TalonFXConfiguration tunedConfigs = createTunedMotorConfig(outputs);
      Slot0Configs slot0 = tunedConfigs.Slot0;
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> rightShooterLeader.getConfigurator().apply(slot0, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> leftShooterLeader.getConfigurator().apply(slot0, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> rightShooterFollower.getConfigurator().apply(slot0, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> leftShooterFollower.getConfigurator().apply(slot0, tunedConfigTimeout)); 
      
      MotionMagicConfigs mmConfigs = tunedConfigs.MotionMagic;
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> rightShooterLeader.getConfigurator().apply(mmConfigs, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> leftShooterLeader.getConfigurator().apply(mmConfigs, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> rightShooterFollower.getConfigurator().apply(mmConfigs, tunedConfigTimeout));
      PhoenixUtil.tryUntilOk(tunedConfigMaxAttempts, () -> leftShooterFollower.getConfigurator().apply(mmConfigs, tunedConfigTimeout));      
    }
  }

  /**
   * Creates TalonFX configuration with latest PID, SVA, and Motion Magic settings updated by outputs.
   * @param outputs
   * @return TalonFX configuration with latest settings.
   */
  private TalonFXConfiguration createTunedMotorConfig(ShooterIOOutputs outputs)
  {
      TalonFXConfiguration configs = new TalonFXConfiguration();
      Slot0Configs slot0Configs = configs.Slot0;
      slot0Configs.kP = outputs.kP;
      slot0Configs.kI = outputs.kI;
      slot0Configs.kD = outputs.kD;
      slot0Configs.kS = outputs.kS;
      slot0Configs.kV = outputs.kV;
      slot0Configs.kA = outputs.kA;
      MotionMagicConfigs mmConfigs = configs.MotionMagic;
      mmConfigs.MotionMagicAcceleration = outputs.kMMAcceleration;
      mmConfigs.MotionMagicJerk = outputs.kMMJerk;
      return configs;
  }

  // =======================UNTESTED/UNUSED METHODS=======================

  /**
   * =======================UNTESTED=======================
   * ========================UNUSED========================
   *
   * @return
   */
  @Override
  public BooleanSupplier leftShooterAtVelocity() {

    return (() ->
        leftShooterLeader.getVelocity().getValueAsDouble()
            > desiredRPS - ShooterConstants.FLYWHEEL_TOLERANCE_RPS);
  }

  /**
   * =======================UNTESTED=======================
   * ========================UNUSED========================
   *
   * @return
   */
  @Override
  public BooleanSupplier bothShootersAtVelocity() {

    return () -> leftShooterAtVelocity().getAsBoolean() && rightShooterAtVelocity().getAsBoolean();
  }
}
