package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kintakeTableKey;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import frc.robot.RobotState.RollerModeState;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Real intake IO implementation using two Kraken/TalonFX roller motors, one Kraken/TalonFX
 * slapdown motor, and active-low slapdown limit sensors.
 *
 * <p>The IO layer reads motor/sensor state and applies requested outputs. The intake subsystem owns
 * the behavior and decides which mode the slapdown and rollers should use.
 */
public class IntakeIOKraken implements IntakeIO {
  private static final int STOW_SLOT = 0;
  private static final int DEPLOY_SLOT = 1;
  private static final int STOW_FULL_SLOT = 2;

  private static final String UPDATE_DEPLOY_CONFIG_NAME = "Update Deploy Configs";

  private static final double INIT_CONFIG_TIMEOUT = 0.250;
  private static final double TUNED_CONFIG_TIMEOUT = 0.100;
  private static final int INIT_CONFIG_MAX_ATTEMPTS = 5;
  private static final int TUNED_CONFIG_MAX_ATTEMPTS = 2;

  private final TalonFX intakeLeft = new TalonFX(IntakeConstants.ROLLER_LEFT);
  private final TalonFX intakeRight = new TalonFX(IntakeConstants.ROLLER_RIGHT);
  private final TalonFX deployMotor = new TalonFX(IntakeConstants.SLAPDOWN_ID);

  private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_SLAPDOWN_DIO_PORT);
  private final DigitalInput bottomLimit =
      new DigitalInput(IntakeConstants.BOTTOM_SLAPDOWN_DIO_PORT);

  private final PositionTorqueCurrentFOC torquePositionControl =
      new PositionTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC torqueRollerControl = new TorqueCurrentFOC(0.0).withDeadband(1.0);

  private final StatusSignal<Angle> deployAngle = deployMotor.getPosition();
  private final StatusSignal<AngularVelocity> deployAngularVelocity = deployMotor.getVelocity();
  private final StatusSignal<Current> deploySupplyCurrent = deployMotor.getSupplyCurrent();
  private final StatusSignal<Current> deployStatorCurrent = deployMotor.getStatorCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = deployMotor.getTorqueCurrent();

  private final StatusSignal<AngularVelocity> intakeLeftRPS = intakeLeft.getVelocity();
  private final StatusSignal<AngularVelocity> intakeRightRPS = intakeRight.getVelocity();
  private final StatusSignal<Current> intakeLeftSupplyCurrent = intakeLeft.getSupplyCurrent();
  private final StatusSignal<Current> intakeRightSupplyCurrent = intakeRight.getSupplyCurrent();
  private final StatusSignal<Current> intakeLeftStatorCurrent = intakeLeft.getStatorCurrent();
  private final StatusSignal<Current> intakeRightStatorCurrent = intakeRight.getStatorCurrent();
  private final StatusSignal<Current> intakeLeftTorqueCurrent = intakeLeft.getTorqueCurrent();
  private final StatusSignal<Current> intakeRightTorqueCurrent = intakeRight.getTorqueCurrent();
  private final StatusSignal<Temperature> intakeLeftTemp = intakeLeft.getDeviceTemp();
  private final StatusSignal<Temperature> intakeRightTemp = intakeRight.getDeviceTemp();

  private int tuneConfigsCreated = 0;
  private double rawDeployPosition = 0.0;
  private double encoderOffset = 0.0;
  private double appliedSlapdownStatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;

  /**
   * Creates the real intake IO layer and configures all motor controllers and status signals.
   *
   * <p>This constructor only configures hardware. The subsystem still controls when the intake moves
   * and which RobotState modes are active.
   */
  public IntakeIOKraken() {
    initializeTuningDashboard();
    configureRollerMotors();
    configureDeployMotor();
    configureStatusSignals();
  }

  /**
   * Initializes dashboard values used by deploy motor tuning.
   *
   * <p>The actual deploy configuration is only applied when tuning mode is enabled and the update
   * boolean is toggled.
   */
  private void initializeTuningDashboard() {
    SmartDashboard.putNumber(kintakeTableKey + "Tune configs created", 0);
    SmartDashboard.putString(kintakeTableKey + "Tune slot0 stow created", "N/A");
    SmartDashboard.putString(kintakeTableKey + "Tune slot1 deploy created", "N/A");
    SmartDashboard.putString(kintakeTableKey + "Tune slot2 stow full created", "N/A");
    SmartDashboard.putString(kintakeTableKey + "Tune MM stow created", "N/A");

    if (Constants.tuningMode) {
      SmartDashboard.putBoolean(kintakeTableKey + UPDATE_DEPLOY_CONFIG_NAME, false);
    }
  }

  /**
   * Applies base configuration to the left and right roller motors.
   *
   * <p>The right roller is inverted relative to the left so both rollers move fuel in the same
   * direction when given the same output.
   */
  private void configureRollerMotors() {
    var intakeLeftConfig = new TalonFXConfiguration();

    intakeLeftConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
    intakeLeftConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
    intakeLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var intakeRightConfig = intakeLeftConfig.clone();
    intakeRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> intakeRight.getConfigurator().apply(intakeRightConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> intakeLeft.getConfigurator().apply(intakeLeftConfig, INIT_CONFIG_TIMEOUT));
  }

  /**
   * Applies base configuration to the slapdown deploy motor.
   *
   * <p>The deploy motor uses three closed-loop slots:
   *
   * <ul>
   *   <li>Slot 0: stow
   *   <li>Slot 1: deploy
   *   <li>Slot 2: stow when full / bump-style positions
   * </ul>
   */
  private void configureDeployMotor() {
    var deployConfig = new TalonFXConfiguration();

    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SLAPDOWN_SUPPLY_CURRENT_LIMIT;
    deployConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.SLAPDOWN_STATOR_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    deployConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        IntakeConstants.PEAK_FORWARD_STATOR_CURRENT_LIMIT;
    deployConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        IntakeConstants.PEAK_REVERSE_STATOR_CURRENT_LIMIT;

    configureStowSlot(deployConfig.Slot0);
    configureDeploySlot(deployConfig.Slot1);
    configureStowFullSlot(deployConfig.Slot2);
    configureMotionMagic(deployConfig.MotionMagic);

    PhoenixUtil.tryUntilOk(
        INIT_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(deployConfig, INIT_CONFIG_TIMEOUT));
  }

  /** Sets status signal update rates and reduces unnecessary CAN bus traffic. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployStatorCurrent,
        deployTorqueCurrent,
        intakeLeftRPS,
        intakeRightRPS,
        intakeLeftSupplyCurrent,
        intakeRightSupplyCurrent,
        intakeLeftStatorCurrent,
        intakeRightStatorCurrent,
        intakeLeftTorqueCurrent,
        intakeRightTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp);
    deployMotor.optimizeBusUtilization();
    intakeLeft.optimizeBusUtilization();
    intakeRight.optimizeBusUtilization();
  }

  /**
   * Refreshes motor status signals, reads limit sensors, and updates the slapdown encoder offset.
   *
   * <p>The slapdown position is reported in adjusted mechanism units where deploy/down is zero and
   * stow/up is {@link IntakeConstants#UP}. The raw encoder value is still logged for debugging.
   *
   * @param inputs container updated with the latest intake hardware state
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployStatorCurrent,
        deployTorqueCurrent,
        intakeLeftRPS,
        intakeRightRPS,
        intakeLeftSupplyCurrent,
        intakeRightSupplyCurrent,
        intakeLeftStatorCurrent,
        intakeRightStatorCurrent,
        intakeLeftTorqueCurrent,
        intakeRightTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp);

    inputs.slapdownVelocity = deployAngularVelocity.getValueAsDouble();
    inputs.slapdownTorqueCurrent = deployTorqueCurrent.getValueAsDouble();
    inputs.slapdownSupplyCurrent = deploySupplyCurrent.getValueAsDouble();
    inputs.slapdownStatorCurrent = deployStatorCurrent.getValueAsDouble();

    inputs.rollerLeftDutyCycle = intakeLeft.get();
    inputs.rollerRightDutyCycle = intakeRight.get();
    inputs.rollerLeftTemperature = intakeLeftTemp.getValueAsDouble();
    inputs.rollerRightTemperature = intakeRightTemp.getValueAsDouble();
    inputs.rollerLeftRPS = intakeLeftRPS.getValueAsDouble();
    inputs.rollerRightRPS = intakeRightRPS.getValueAsDouble();
    inputs.rollerLeftSupplyCurrent = intakeLeftSupplyCurrent.getValueAsDouble();
    inputs.rollerRightSupplyCurrent = intakeRightSupplyCurrent.getValueAsDouble();
    inputs.rollerLeftStatorCurrent = intakeLeftStatorCurrent.getValueAsDouble();
    inputs.rollerRightStatorCurrent = intakeRightStatorCurrent.getValueAsDouble();
    inputs.rollerLeftTorqueCurrent = intakeLeftTorqueCurrent.getValueAsDouble();
    inputs.rollerRightTorqueCurrent = intakeRightTorqueCurrent.getValueAsDouble();

    inputs.slapdownDown = isBottomLimitTripped();
    inputs.slapdownUp = isTopLimitTripped();

    updateSlapdownEncoderOffset(inputs);
  }

  /**
   * Applies roller, slapdown current limit, and slapdown motion outputs based on current RobotState
   * modes.
   *
   * @param outputs latest requested intake outputs from the subsystem
   */
  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (Constants.tuningMode) {
      tuneDeployMotorConfigs(outputs);
    }

    applySlapdownCurrentLimit(outputs.slapdownStatorCurrentLimit);
    applyRollerOutput(outputs.appliedRollerSpeed);
    applySlapdownOutput(outputs);
  }

  /**
   * Applies roller output using either torque-current mode or duty-cycle mode.
   *
   * <p>Torque-current mode is only used for normal pickup speed when RobotState requests it. Other
   * speeds use normal duty-cycle output so reverse/manual commands behave as expected.
   *
   * @param rollerSpeed requested roller speed
   */
  private void applyRollerOutput(double rollerSpeed) {
    boolean useTorqueMode =
      rollerSpeed > 0.0 && RobotState.getRollerMode() == RollerModeState.TORQUE_CURRENT;

    if (useTorqueMode) {
      double torqueOutput = IntakeConstants.rollerBoostTorqueCurrent.getAsDouble();
      intakeLeft.setControl(torqueRollerControl.withOutput(torqueOutput));
      intakeRight.setControl(torqueRollerControl.withOutput(torqueOutput));
      return;
    }

    intakeLeft.set(rollerSpeed);
    intakeRight.set(rollerSpeed);
  }

  /**
   * Applies the requested slapdown output based on the current slapdown mode.
   *
   * @param outputs latest requested intake outputs from the subsystem
   */
  private void applySlapdownOutput(IntakeIOOutputs outputs) {
    switch (RobotState.getSlapdownMode()) {
      case DEPLOY_POSITION:
        slapToPosition(DEPLOY_SLOT, outputs.desiredSlapdownPosition, outputs.deployFF);
        break;

      case STOW_POSITION:
        slapToPosition(STOW_SLOT, outputs.desiredSlapdownPosition, outputs.stowFF);
        break;

      case BUMP_POSITION:
        slapToPosition(STOW_FULL_SLOT, outputs.desiredSlapdownPosition, outputs.stowFullFF);
        break;

      case SPEED:
        deployMotor.set(outputs.appliedSlapdownSpeed);
        break;

      case OFF:
        deployMotor.stopMotor();
        break;

      default:
        Logger.recordOutput(
            kintakeTableKey + "UnknownSlapdownMode", RobotState.getSlapdownMode().toString());
        deployMotor.stopMotor();
        break;
    }
  }

  /**
   * Applies a slapdown stator current limit when the requested value changes.
   *
   * <p>This allows slow shooting stow to use a lower current limit without repeatedly reconfiguring
   * the motor controller every loop.
   *
   * @param statorCurrentLimit requested slapdown stator current limit in amps
   */
  private void applySlapdownCurrentLimit(double statorCurrentLimit) {
    if (Math.abs(statorCurrentLimit - appliedSlapdownStatorCurrentLimit) < 1e-6) {
      return;
    }

    appliedSlapdownStatorCurrentLimit = statorCurrentLimit;

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(IntakeConstants.SLAPDOWN_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(appliedSlapdownStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);

    var status = deployMotor.getConfigurator().apply(currentLimits);

    Logger.recordOutput(kintakeTableKey + "SlapdownCurrentLimit/ApplyStatus", status.toString());
    Logger.recordOutput(
        kintakeTableKey + "SlapdownCurrentLimit/Stator", appliedSlapdownStatorCurrentLimit);
  }

  /**
   * Runs the slapdown to a requested position using the selected TalonFX slot.
   *
   * <p>The subsystem works in offset-adjusted slapdown positions. The IO layer adds
   * {@code rawDeployPosition} back in so the motor controller receives the matching raw encoder
   * target.
   *
   * @param slot TalonFX slot to use
   * @param position requested adjusted slapdown position
   * @param feedForward extra feedforward applied to the position request
   */
  private void slapToPosition(int slot, double position, double feedForward) {
    deployMotor.setControl(
        torquePositionControl
            .withPosition(position + rawDeployPosition)
            .withSlot(slot)
            .withFeedForward(feedForward));
  }

  /**
   * Updates the adjusted slapdown encoder position using the active-low limit sensors.
   *
   * <p>When the deploy/down sensor is tripped, the adjusted position is reset to
   * {@link IntakeConstants#DOWN}. When the stow/up sensor is tripped, the adjusted position is reset
   * to {@link IntakeConstants#UP}.
   *
   * @param inputs input container receiving raw and adjusted positions
   */
  private void updateSlapdownEncoderOffset(IntakeIOInputs inputs) {
    double rawAngle = deployAngle.getValueAsDouble();

    if (inputs.slapdownDown) {
      rawDeployPosition = rawAngle;
      encoderOffset = -rawAngle;
    } else if (inputs.slapdownUp) {
      rawDeployPosition = rawAngle - IntakeConstants.UP;
      encoderOffset = -(rawAngle - IntakeConstants.UP);
    }

    inputs.slapdownEncoderOffset = encoderOffset;
    inputs.slapdownRawPosition = rawAngle;
    inputs.slapdownPosition = rawAngle + encoderOffset;

    Logger.recordOutput(kintakeTableKey + "RawDeployPosition", rawDeployPosition);
  }

  /**
   * Returns whether the bottom/deployed slapdown limit sensor is active.
   *
   * @return true when the slapdown is at the deployed/down sensor
   */
  private boolean isBottomLimitTripped() {
    return bottomLimit.get() == IntakeConstants.SLAPDOWN_LIMIT_TRIPPED;
  }

  /**
   * Returns whether the top/stowed slapdown limit sensor is active.
   *
   * @return true when the slapdown is at the stowed/up sensor
   */
  private boolean isTopLimitTripped() {
    return topLimit.get() == IntakeConstants.SLAPDOWN_LIMIT_TRIPPED;
  }

  /**
   * Applies deploy motor tuning values when the dashboard update boolean is toggled.
   *
   * <p>This avoids reapplying TalonFX configs every loop while still allowing tuning values to be
   * pushed during testing.
   *
   * @param outputs latest requested intake outputs containing tunable gains
   */
  private void tuneDeployMotorConfigs(IntakeIOOutputs outputs) {
    if (!SmartDashboard.getBoolean(kintakeTableKey + UPDATE_DEPLOY_CONFIG_NAME, true)) {
      return;
    }

    SmartDashboard.putBoolean(kintakeTableKey + UPDATE_DEPLOY_CONFIG_NAME, false);

    TalonFXConfiguration tunedConfigs = createTunedDeployMotorConfig(outputs);

    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(tunedConfigs.Slot0, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(tunedConfigs.Slot1, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(tunedConfigs.Slot2, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(tunedConfigs.MotionMagic, TUNED_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(
        TUNED_CONFIG_MAX_ATTEMPTS,
        () -> deployMotor.getConfigurator().apply(tunedConfigs.TorqueCurrent, TUNED_CONFIG_TIMEOUT));
  }

  /**
   * Creates a deploy motor configuration from the latest tunable outputs.
   *
   * @param outputs latest requested intake outputs containing tunable gains
   * @return TalonFX configuration containing updated deploy motor tuning values
   */
  private TalonFXConfiguration createTunedDeployMotorConfig(IntakeIOOutputs outputs) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configureStowSlot(configs.Slot0, outputs);
    configureDeploySlot(configs.Slot1, outputs);
    configureStowFullSlot(configs.Slot2, outputs);

    configs.MotionMagic.MotionMagicAcceleration = outputs.stowMMAcceleration;
    configs.MotionMagic.MotionMagicJerk = outputs.stowMMJerk;

    configs.TorqueCurrent.PeakForwardTorqueCurrent =
        IntakeConstants.peakForwardStatorCurrentLimit.get();
    configs.TorqueCurrent.PeakReverseTorqueCurrent =
        IntakeConstants.peakReverseStatorCurrentLimit.get();

    logCreatedTuningConfig(configs);

    return configs;
  }

  /** Applies default stow gains to slot 0. */
  private void configureStowSlot(Slot0Configs slot0) {
    slot0.kP = IntakeConstants.StowConfigs.kP;
    slot0.kI = IntakeConstants.StowConfigs.kI;
    slot0.kD = IntakeConstants.StowConfigs.kD;
    slot0.kG = IntakeConstants.StowConfigs.kG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies tunable stow gains to slot 0. */
  private void configureStowSlot(Slot0Configs slot0, IntakeIOOutputs outputs) {
    slot0.kP = outputs.stowKP;
    slot0.kI = outputs.stowKI;
    slot0.kD = outputs.stowKD;
    slot0.kG = outputs.stowKG;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies default deploy gains to slot 1. */
  private void configureDeploySlot(Slot1Configs slot1) {
    slot1.kP = IntakeConstants.DeployConfigs.kP;
    slot1.kI = IntakeConstants.DeployConfigs.kI;
    slot1.kD = IntakeConstants.DeployConfigs.kD;
    slot1.kG = IntakeConstants.DeployConfigs.kG;
    slot1.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies tunable deploy gains to slot 1. */
  private void configureDeploySlot(Slot1Configs slot1, IntakeIOOutputs outputs) {
    slot1.kP = outputs.deployKP;
    slot1.kI = outputs.deployKI;
    slot1.kD = outputs.deployKD;
    slot1.kG = outputs.deployKG;
    slot1.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies default full-stow gains to slot 2. */
  private void configureStowFullSlot(Slot2Configs slot2) {
    slot2.kP = IntakeConstants.StowFullConfigs.kP;
    slot2.kI = IntakeConstants.StowFullConfigs.kI;
    slot2.kD = IntakeConstants.StowFullConfigs.kD;
    slot2.kG = IntakeConstants.StowFullConfigs.kG;
    slot2.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies tunable full-stow gains to slot 2. */
  private void configureStowFullSlot(Slot2Configs slot2, IntakeIOOutputs outputs) {
    slot2.kP = outputs.stowFullKP;
    slot2.kI = outputs.stowFullKI;
    slot2.kD = outputs.stowFullKD;
    slot2.kG = outputs.stowFullKG;
    slot2.GravityType = GravityTypeValue.Arm_Cosine;
  }

  /** Applies default Motion Magic values for slapdown stowing. */
  private void configureMotionMagic(MotionMagicConfigs motionMagic) {
    motionMagic.MotionMagicAcceleration = IntakeConstants.StowConfigs.kmmAcceleration;
    motionMagic.MotionMagicJerk = IntakeConstants.StowConfigs.kmmJerk;
  }

  /**
   * Logs the generated tuning config for debugging.
   *
   * @param configs generated deploy motor configuration
   */
  private void logCreatedTuningConfig(TalonFXConfiguration configs) {
    SmartDashboard.putNumber(kintakeTableKey + "Tune configs created", ++tuneConfigsCreated);
    SmartDashboard.putString(kintakeTableKey + "Tune slot0 stow created", configs.Slot0.toString());
    SmartDashboard.putString(
        kintakeTableKey + "Tune slot1 deploy created", configs.Slot1.toString());
    SmartDashboard.putString(
        kintakeTableKey + "Tune slot2 stow full created", configs.Slot2.toString());
    SmartDashboard.putString(
        kintakeTableKey + "Tune MM stow created", configs.MotionMagic.toString());
  }
}