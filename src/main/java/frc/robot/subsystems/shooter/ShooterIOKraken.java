package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Real shooter IO implementation using four Kraken/TalonFX motors.
 *
 * <p>The right leader is directly controlled. The right follower follows the right leader, and both
 * left motors follow the right leader using the current follower configuration from the original
 * robot code. CTRE velocity signals are rotations per second, but the subsystem and logs use RPM.
 */
public class ShooterIOKraken implements ShooterIO {
  private static final String CURRENT_CONTROL_MODE = "ControlMode";
  private static final double INIT_CONFIG_TIMEOUT = 0.250;
  private static final double TUNED_CONFIG_TIMEOUT = 0.100;
  private static final int INIT_CONFIG_MAX_ATTEMPTS = 5;
  private static final int TUNED_CONFIG_MAX_ATTEMPTS = 2;

  private final TalonFX rightShooterLeader =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);
  private final TalonFX leftShooterLeader =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);

  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);
  private final MotionMagicVelocityVoltage motionMagicVelocityControl =
      new MotionMagicVelocityVoltage(0.0).withSlot(0);

  private final StatusSignal<AngularVelocity> rightLeaderVelocity =
      rightShooterLeader.getVelocity();
  private final StatusSignal<AngularVelocity> rightFollowerVelocity =
      rightShooterFollower.getVelocity();
  private final StatusSignal<AngularVelocity> leftLeaderVelocity =
      leftShooterLeader.getVelocity();
  private final StatusSignal<AngularVelocity> leftFollowerVelocity =
      leftShooterFollower.getVelocity();

  private final StatusSignal<Voltage> rightLeaderAppliedVolts =
      rightShooterLeader.getMotorVoltage();
  private final StatusSignal<Voltage> rightFollowerAppliedVolts =
      rightShooterFollower.getMotorVoltage();
  private final StatusSignal<Voltage> leftLeaderAppliedVolts =
      leftShooterLeader.getMotorVoltage();
  private final StatusSignal<Voltage> leftFollowerAppliedVolts =
      leftShooterFollower.getMotorVoltage();

  private final StatusSignal<Temperature> rightLeaderTemperature =
      rightShooterLeader.getDeviceTemp();
  private final StatusSignal<Temperature> rightFollowerTemperature =
      rightShooterFollower.getDeviceTemp();
  private final StatusSignal<Temperature> leftLeaderTemperature =
      leftShooterLeader.getDeviceTemp();
  private final StatusSignal<Temperature> leftFollowerTemperature =
      leftShooterFollower.getDeviceTemp();

  private final StatusSignal<Current> rightLeaderSupplyCurrent =
      rightShooterLeader.getSupplyCurrent();
  private final StatusSignal<Current> rightFollowerSupplyCurrent =
      rightShooterFollower.getSupplyCurrent();
  private final StatusSignal<Current> leftLeaderSupplyCurrent =
      leftShooterLeader.getSupplyCurrent();
  private final StatusSignal<Current> leftFollowerSupplyCurrent =
      leftShooterFollower.getSupplyCurrent();

  private final StatusSignal<Current> rightLeaderStatorCurrent =
      rightShooterLeader.getStatorCurrent();
  private final StatusSignal<Current> rightFollowerStatorCurrent =
      rightShooterFollower.getStatorCurrent();
  private final StatusSignal<Current> leftLeaderStatorCurrent =
      leftShooterLeader.getStatorCurrent();
  private final StatusSignal<Current> leftFollowerStatorCurrent =
      leftShooterFollower.getStatorCurrent();

  private final StatusSignal<Current> rightLeaderTorqueCurrent =
      rightShooterLeader.getTorqueCurrent();
  private final StatusSignal<Current> rightFollowerTorqueCurrent =
      rightShooterFollower.getTorqueCurrent();
  private final StatusSignal<Current> leftLeaderTorqueCurrent =
      leftShooterLeader.getTorqueCurrent();
  private final StatusSignal<Current> leftFollowerTorqueCurrent =
      leftShooterFollower.getTorqueCurrent();

  private double desiredVelocityRPM = 0.0;
  private int tuneConfigsCreated = 0;

  /**
   * Creates the real shooter IO layer and configures all shooter motor controllers.
   */
  public ShooterIOKraken() {
    initializeTuningDashboard();
    configureShooterMotors();
    configureFollowers();
    configureStatusSignals();
  }

  /** Initializes dashboard entries used to push shooter tuning configs. */
  private void initializeTuningDashboard() {
    SmartDashboard.putString(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, "N/A");
    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Tune configs created", tuneConfigsCreated);
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune slot0 created", "N/A");
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune MM created", "N/A");
  }

  /** Applies base motor configs to all shooter motors. */
  private void configureShooterMotors() {
    TalonFXConfiguration rightConfig = createRightShooterConfig();
    TalonFXConfiguration leftConfig = createLeftShooterConfig();

    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterLeader.getConfigurator().apply(rightConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterFollower.getConfigurator().apply(rightConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterLeader.getConfigurator().apply(leftConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterFollower.getConfigurator().apply(leftConfig, INIT_CONFIG_TIMEOUT));
  }

  /**
   * Creates the configuration for the right-side shooter motors.
   *
   * @return right-side TalonFX configuration
   */
  private TalonFXConfiguration createRightShooterConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configureSlot0(config.Slot0);
    configureMotionMagic(config.MotionMagic);

    return config;
  }

  /**
   * Creates the configuration for the left-side shooter motors.
   *
   * @return left-side TalonFX configuration
   */
  private TalonFXConfiguration createLeftShooterConfig() {
    TalonFXConfiguration config = createRightShooterConfig();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return config;
  }

  /** Configures the shooter motor follower relationships. */
  private void configureFollowers() {
    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterLeader.setControl(new StrictFollower(rightShooterLeader.getDeviceID()));
    leftShooterFollower.setControl(new StrictFollower(rightShooterLeader.getDeviceID()));
  }

  /** Sets status signal update rates and reduces unnecessary CAN bus traffic. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        rightLeaderVelocity,
        rightFollowerVelocity,
        leftLeaderVelocity,
        leftFollowerVelocity,
        rightLeaderAppliedVolts,
        rightFollowerAppliedVolts,
        leftLeaderAppliedVolts,
        leftFollowerAppliedVolts,
        rightLeaderTemperature,
        rightFollowerTemperature,
        leftLeaderTemperature,
        leftFollowerTemperature,
        rightLeaderSupplyCurrent,
        rightFollowerSupplyCurrent,
        leftLeaderSupplyCurrent,
        leftFollowerSupplyCurrent,
        rightLeaderStatorCurrent,
        rightFollowerStatorCurrent,
        leftLeaderStatorCurrent,
        leftFollowerStatorCurrent,
        rightLeaderTorqueCurrent,
        rightFollowerTorqueCurrent,
        leftLeaderTorqueCurrent,
        leftFollowerTorqueCurrent);

    rightShooterLeader.optimizeBusUtilization();
    rightShooterFollower.optimizeBusUtilization();
    leftShooterLeader.optimizeBusUtilization();
    leftShooterFollower.optimizeBusUtilization();
  }

  /**
   * Refreshes shooter motor status signals.
   *
   * @param inputs container updated with the latest shooter hardware state
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rightLeaderVelocity,
        rightFollowerVelocity,
        leftLeaderVelocity,
        leftFollowerVelocity,
        rightLeaderAppliedVolts,
        rightFollowerAppliedVolts,
        leftLeaderAppliedVolts,
        leftFollowerAppliedVolts,
        rightLeaderTemperature,
        rightFollowerTemperature,
        leftLeaderTemperature,
        leftFollowerTemperature,
        rightLeaderSupplyCurrent,
        rightFollowerSupplyCurrent,
        leftLeaderSupplyCurrent,
        leftFollowerSupplyCurrent,
        rightLeaderStatorCurrent,
        rightFollowerStatorCurrent,
        leftLeaderStatorCurrent,
        leftFollowerStatorCurrent,
        rightLeaderTorqueCurrent,
        rightFollowerTorqueCurrent,
        leftLeaderTorqueCurrent,
        leftFollowerTorqueCurrent);

    inputs.rightLeaderConnected = rightShooterLeader.isConnected();
    inputs.rightFollowerConnected = rightShooterFollower.isConnected();
    inputs.leftLeaderConnected = leftShooterLeader.isConnected();
    inputs.leftFollowerConnected = leftShooterFollower.isConnected();

    inputs.rightLeaderVelocityRPM = rpsToRpm(rightLeaderVelocity.getValueAsDouble());
    inputs.rightFollowerVelocityRPM = rpsToRpm(rightFollowerVelocity.getValueAsDouble());
    inputs.leftLeaderVelocityRPM = rpsToRpm(leftLeaderVelocity.getValueAsDouble());
    inputs.leftFollowerVelocityRPM = rpsToRpm(leftFollowerVelocity.getValueAsDouble());

    inputs.rightLeaderAppliedVolts = rightLeaderAppliedVolts.getValueAsDouble();
    inputs.rightFollowerAppliedVolts = rightFollowerAppliedVolts.getValueAsDouble();
    inputs.leftLeaderAppliedVolts = leftLeaderAppliedVolts.getValueAsDouble();
    inputs.leftFollowerAppliedVolts = leftFollowerAppliedVolts.getValueAsDouble();

    inputs.rightLeaderTemperature = rightLeaderTemperature.getValueAsDouble();
    inputs.rightFollowerTemperature = rightFollowerTemperature.getValueAsDouble();
    inputs.leftLeaderTemperature = leftLeaderTemperature.getValueAsDouble();
    inputs.leftFollowerTemperature = leftFollowerTemperature.getValueAsDouble();

    inputs.rightLeaderSupplyCurrent = rightLeaderSupplyCurrent.getValueAsDouble();
    inputs.rightFollowerSupplyCurrent = rightFollowerSupplyCurrent.getValueAsDouble();
    inputs.leftLeaderSupplyCurrent = leftLeaderSupplyCurrent.getValueAsDouble();
    inputs.leftFollowerSupplyCurrent = leftFollowerSupplyCurrent.getValueAsDouble();

    inputs.rightLeaderStatorCurrent = rightLeaderStatorCurrent.getValueAsDouble();
    inputs.rightFollowerStatorCurrent = rightFollowerStatorCurrent.getValueAsDouble();
    inputs.leftLeaderStatorCurrent = leftLeaderStatorCurrent.getValueAsDouble();
    inputs.leftFollowerStatorCurrent = leftFollowerStatorCurrent.getValueAsDouble();

    inputs.rightLeaderTorqueCurrent = rightLeaderTorqueCurrent.getValueAsDouble();
    inputs.rightFollowerTorqueCurrent = rightFollowerTorqueCurrent.getValueAsDouble();
    inputs.leftLeaderTorqueCurrent = leftLeaderTorqueCurrent.getValueAsDouble();
    inputs.leftFollowerTorqueCurrent = leftFollowerTorqueCurrent.getValueAsDouble();

    inputs.shooterAtVelocity = rightShooterAtVelocity().getAsBoolean();
  }

  /**
   * Applies shooter output based on the current shooter mode.
   *
   * @param outputs latest requested shooter outputs from the subsystem
   */
  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    desiredVelocityRPM = outputs.desiredVelocityRPM;

    switch (RobotState.getShooterMode()) {
      case ON:
        applyVelocityOutput(outputs, outputs.kMMShootAcceleration);
        break;

      case IDLE:
        applyVelocityOutput(outputs, outputs.kMMAcceleration);
        break;

      case DUTYCYCLE:
        applyDutyCycleOutput(outputs);
        break;

      case OFF:
        stopShooter();
        break;

      default:
        Logger.recordOutput(
            SHOOTER_TABLE_KEY + "UnknownMode", RobotState.getShooterMode().toString());
        stopShooter();
        break;
    }
  }

  /** Applies closed-loop velocity control to the shooter leader motor. */
  private void applyVelocityOutput(ShooterIOOutputs outputs, double motionMagicAcceleration) {
    double desiredRps = rpmToRps(outputs.desiredVelocityRPM);

    Logger.recordOutput(SHOOTER_TABLE_KEY + "DesiredRPSInternal", desiredRps);
    Logger.recordOutput(SHOOTER_TABLE_KEY + "ActiveMMAcceleration", motionMagicAcceleration);

    if (outputs.useMotionMagic) {
      rightShooterLeader.setControl(
          motionMagicVelocityControl
              .withVelocity(desiredRps)
              .withAcceleration(motionMagicAcceleration));
      Logger.recordOutput(
          SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE,
          MotionMagicVelocityVoltage.class.getSimpleName());
      return;
    }

    rightShooterLeader.setControl(velocityControl.withVelocity(desiredRps));
    Logger.recordOutput(
        SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE,
        VelocityTorqueCurrentFOC.class.getSimpleName());
  }

  /** Applies open-loop duty-cycle control to the shooter leader motor. */
  private void applyDutyCycleOutput(ShooterIOOutputs outputs) {
    rightShooterLeader.set(outputs.desiredDutyCycle);
    Logger.recordOutput(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, "DutyCycle");
  }

  /** Stops the shooter leader motor. Followers are stopped through follower behavior. */
  private void stopShooter() {
    rightShooterLeader.stopMotor();
    Logger.recordOutput(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, "Off");
  }

  @Override
  public BooleanSupplier rightShooterAtVelocity() {
    return rightShooterAtVelocityRPM(() -> desiredVelocityRPM);
  }

  @Override
  public BooleanSupplier rightShooterAtVelocityRPM(DoubleSupplier targetRPM) {
    return () ->
        Math.abs(rpsToRpm(rightLeaderVelocity.getValueAsDouble()) - targetRPM.getAsDouble())
            < ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
  }

  @Override
  public BooleanSupplier rightShooterBelowCoastRPM() {
    return () ->
        rpsToRpm(rightLeaderVelocity.getValueAsDouble())
            < ShooterConstants.coastRPM.getAsDouble() + ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
  }

  /**
   * Applies tunable shooter configs when the dashboard update boolean is toggled.
   *
   * @param outputs latest requested shooter outputs containing tunable values
   */
  @Override
  public void tuneMotorConfigs(ShooterIOOutputs outputs) {
    if (!SmartDashboard.getBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false)) {
      return;
    }

    SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);

    TalonFXConfiguration tunedConfigs = createTunedMotorConfig(outputs);
    Slot0Configs slot0 = tunedConfigs.Slot0;
    MotionMagicConfigs motionMagic = tunedConfigs.MotionMagic;

    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterLeader.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterFollower.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterLeader.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterFollower.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));

    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterLeader.getConfigurator().apply(motionMagic, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> rightShooterFollower.getConfigurator().apply(motionMagic, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterLeader.getConfigurator().apply(motionMagic, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> leftShooterFollower.getConfigurator().apply(motionMagic, TUNED_CONFIG_TIMEOUT));
  }

  /**
   * Creates a shooter motor configuration from the latest tunable outputs.
   *
   * @param outputs latest requested shooter outputs containing tunable values
   * @return TalonFX config with updated slot and Motion Magic values
   */
  private TalonFXConfiguration createTunedMotorConfig(ShooterIOOutputs outputs) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = outputs.kP;
    configs.Slot0.kI = outputs.kI;
    configs.Slot0.kD = outputs.kD;
    configs.Slot0.kS = outputs.kS;
    configs.Slot0.kV = outputs.kV;
    configs.Slot0.kA = outputs.kA;

    configs.MotionMagic.MotionMagicAcceleration = outputs.kMMAcceleration;
    configs.MotionMagic.MotionMagicJerk = outputs.kMMJerk;

    logCreatedTuningConfig(configs);

    return configs;
  }

  /** Applies default slot 0 gains to a shooter motor config. */
  private void configureSlot0(Slot0Configs slot0) {
    slot0.kP = ShooterConstants.kP.getAsDouble();
    slot0.kI = ShooterConstants.kI.getAsDouble();
    slot0.kD = ShooterConstants.kD.getAsDouble();
    slot0.kS = ShooterConstants.kS.getAsDouble();
    slot0.kV = ShooterConstants.kV.getAsDouble();
    slot0.kA = ShooterConstants.kA.getAsDouble();
  }

  /** Applies default Motion Magic velocity values to a shooter motor config. */
  private void configureMotionMagic(MotionMagicConfigs motionMagic) {
    motionMagic.MotionMagicAcceleration = ShooterConstants.kMMAcceleration.getAsDouble();
    motionMagic.MotionMagicJerk = ShooterConstants.kMMJerk.getAsDouble();
  }

  /** Logs a generated tuning config for debugging. */
  private void logCreatedTuningConfig(TalonFXConfiguration configs) {
    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Tune configs created", ++tuneConfigsCreated);
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune slot0 created", configs.Slot0.toString());
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune MM created", configs.MotionMagic.toString());
  }

  private static double rpmToRps(double rpm) {
    return rpm / 60.0;
  }

  private static double rpsToRpm(double rps) {
    return rps * 60.0;
  }
}