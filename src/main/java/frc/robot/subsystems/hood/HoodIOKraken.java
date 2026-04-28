package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_TABLE_KEY;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Real hood IO implementation using a Kraken/TalonFX and a bottom DIO limit sensor.
 *
 * <p>The bottom limit sensor is active low. This class exposes both the raw DIO value and a readable
 * "tripped" value where true means the sensor is active.
 */
public class HoodIOKraken implements HoodIO {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
  private final DigitalInput bottomLimitSensor = new DigitalInput(HoodConstants.HOOD_DIO_PORT);

  private final Debouncer bottomLimitSensorDebouncer =
      new Debouncer(HoodConstants.LIMIT_SENSOR_DEBOUNCE_TIME, DebounceType.kRising);

  private final Timer stoppedTimer = new Timer();

  private final Slot0Configs slot0 = new Slot0Configs();
  private final Slot1Configs slot1 = new Slot1Configs();

  private final PositionTorqueCurrentFOC positionTorqueControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

  private double appliedSupplyCurrentLimit = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
  private double appliedStatorCurrentLimit = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;

  private double angleBeforeAppliedZero = 0.0;
  private int acceptedZeroCounter = 0;
  private int slot0AppliedCount = 0;
  private int slot1AppliedCount = 0;

  /**
   * Creates the real hood IO layer and configures the motor controller.
   *
   * <p>This constructor only sets up hardware configuration. The subsystem still controls when the
   * hood moves, when zero is accepted, and what output mode is active.
   */
  public HoodIOKraken() {
    configureMotor();
    configurePidSlots();
    configureStatusSignals();
  }

  /**
   * Applies the base TalonFX configuration for the hood motor.
   *
   * <p>This includes current limits, brake mode, motor inversion, and the sensor-to-mechanism ratio.
   */
  private void configureMotor() {
    var hoodConfig = new TalonFXConfiguration();

    hoodConfig.CurrentLimits.SupplyCurrentLimit = appliedSupplyCurrentLimit;
    hoodConfig.CurrentLimits.StatorCurrentLimit = appliedStatorCurrentLimit;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
  }

  /**
   * Configures the two closed-loop PID slots.
   *
   * <p>Slot 0 is used when moving upward. Slot 1 is used when moving downward so the hood can have
   * different gains depending on direction.
   */
  private void configurePidSlots() {
    slot0.kP = HoodConstants.HOOD_UP_KP;
    slot0.kI = HoodConstants.HOOD_UP_KI;
    slot0.kD = HoodConstants.HOOD_UP_KD;
    slot0.kS = HoodConstants.HOOD_UP_KS;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hoodMotor.getConfigurator().apply(slot0);

    slot1.kP = HoodConstants.HOOD_DOWN_KP;
    slot1.kI = HoodConstants.HOOD_DOWN_KI;
    slot1.kD = HoodConstants.HOOD_DOWN_KD;
    slot1.kS = HoodConstants.HOOD_DOWN_KS;
    slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hoodMotor.getConfigurator().apply(slot1);
  }

  /** Sets status signal update rates and reduces unnecessary CAN bus traffic. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        hoodAppliedVolts,
        hoodAngle,
        hoodAngularVelocity,
        hoodSupplyCurrent,
        hoodStatorCurrent,
        hoodTorqueCurrent,
        hoodTemperature);

    hoodMotor.optimizeBusUtilization();
  }

  /**
   * Refreshes all TalonFX status signals and reads the bottom limit sensor.
   *
   * <p>{@code hoodLimitRaw} is the direct DIO reading. {@code hoodLimitTripped} converts that raw
   * value into the readable subsystem meaning where true means the sensor is active.
   *
   * @param inputs container updated with the latest hood hardware state
   */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodAppliedVolts,
        hoodAngle,
        hoodAngularVelocity,
        hoodSupplyCurrent,
        hoodStatorCurrent,
        hoodTorqueCurrent,
        hoodTemperature);

    inputs.hoodSpeed = hoodAngularVelocity.getValueAsDouble();
    inputs.hoodAngle = hoodAngle.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodStatorCurrent = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodTemperature = hoodTemperature.getValueAsDouble();

    inputs.hoodLimitRaw = bottomLimitSensor.get();
    inputs.hoodLimitTripped = isBottomLimitSensorTripped();
    inputs.hoodLimitSet = inputs.hoodLimitTripped;
  }

  /**
   * Applies the requested hood output mode to the motor controller.
   *
   * <p>Position mode uses closed-loop TalonFX control and selects a PID slot based on travel
   * direction. Speed mode uses open-loop percent output.
   *
   * @param outputs latest requested hood outputs from the subsystem
   */
  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    Logger.recordOutput(HOOD_TABLE_KEY + "DesiredAngle", outputs.desiredHoodAngle);
    Logger.recordOutput(HOOD_TABLE_KEY + "DesiredSpeed", outputs.desiredHoodSpeed);

    if (outputs.mode == HoodMode.POSITION) {
      applyPositionOutput(outputs);
    } else {
      applySpeedOutput(outputs);
    }
  }

  /**
   * Applies a closed-loop hood position request.
   *
   * <p>The selected PID slot depends on direction: slot 0 for moving up and slot 1 for moving down.
   *
   * @param outputs latest requested hood outputs
   */
  private void applyPositionOutput(HoodIOOutputs outputs) {
    if (Constants.Tuning.HOOD) {
      applyTunableSlotConfigs(outputs);
    }

    double currentAngle = hoodAngle.getValueAsDouble();
    boolean movingDown = outputs.desiredHoodAngle < currentAngle;
    int selectedSlot = movingDown ? 1 : 0;

    Logger.recordOutput(HOOD_TABLE_KEY + "Position/SelectedPidSlot", selectedSlot);
    Logger.recordOutput(HOOD_TABLE_KEY + "Position/CurrentAngle", currentAngle);
    Logger.recordOutput(HOOD_TABLE_KEY + "Position/DesiredAngle", outputs.desiredHoodAngle);
    Logger.recordOutput(HOOD_TABLE_KEY + "Position/MovingDown", movingDown);

    hoodMotor.setControl(
        positionTorqueControl.withPosition(outputs.desiredHoodAngle).withSlot(selectedSlot));
  }

  /**
   * Applies an open-loop hood speed request.
   *
   * <p>The speed is clamped as a final safety check before sending it to the motor controller.
   *
   * @param outputs latest requested hood outputs
   */
  private void applySpeedOutput(HoodIOOutputs outputs) {
    double speed =
        MathUtil.clamp(
            outputs.desiredHoodSpeed,
            -HoodConstants.HOOD_MAX_SPEED,
            HoodConstants.HOOD_MAX_SPEED);

    hoodMotor.set(speed);
  }

  /**
   * Seeds the TalonFX mechanism position to zero.
   *
   * <p>This does not move the mechanism. The subsystem should only call this after the hood has
   * reached the zero hard stop or after a trusted manual zero command.
   */
  @Override
  public void zeroHood() {
    angleBeforeAppliedZero = hoodAngle.getValueAsDouble();
    hoodMotor.setPosition(0.0);

    acceptedZeroCounter++;

    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/AngleBeforeAppliedZero", angleBeforeAppliedZero);
    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/AcceptedZeroCount", acceptedZeroCounter);
  }

  /**
   * Returns whether the current encoder position is close to zero.
   *
   * <p>This checks encoder position only. It does not prove the hood is physically touching the zero
   * hard stop. Use {@link #isHoodAtTrueZero()} when zeroing the mechanism.
   *
   * @return supplier that is true when the encoder position is within the configured tolerance
   */
  @Override
  public BooleanSupplier isHoodWithinZeroTolerance() {
    return () -> Math.abs(hoodAngle.getValueAsDouble()) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  /**
   * Returns whether the hood should be treated as physically at zero.
   *
   * <p>The limit sensor is the primary zero detector. If the sensor does not trip, the fallback
   * accepts zero only when the hood appears stalled against the hard stop.
   */
  @Override
  public boolean isHoodAtTrueZero() {
    boolean zeroSensorTripped =
        bottomLimitSensorDebouncer.calculate(isBottomLimitSensorTripped());

    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/LimitSensorTripped", zeroSensorTripped);

    if (zeroSensorTripped) {
      resetHoodTimer();
      return true;
    }

    return hasHoodStoppedOverTime(HoodConstants.MIN_STATIONARY_DURATION);
  }

  /**
   * Returns whether the hood has appeared stalled for the requested duration.
   *
   * <p>For zeroing fallback, "stopped" means the hood has low velocity and elevated current. Low
   * velocity alone is not enough because the zeroing speed is intentionally very slow.
   */
  @Override
  public boolean hasHoodStoppedOverTime(double minStationaryDuration) {
    boolean stopped = hasHoodStopped();

    if (!stopped) {
      resetHoodTimer();
      Logger.recordOutput(HOOD_TABLE_KEY + "Zero/StoppedOverTime", false);
      Logger.recordOutput(HOOD_TABLE_KEY + "Zero/StoppedTimer", 0.0);
      return false;
    }

    stoppedTimer.start();

    boolean stoppedOverTime = stoppedTimer.hasElapsed(minStationaryDuration);

    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/StoppedOverTime", stoppedOverTime);
    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/StoppedTimer", stoppedTimer.get());

    if (stoppedOverTime) {
      resetHoodTimer();
    }

    return stoppedOverTime;
  }

  /**
   * Resets the timer used by the fallback zero detector.
   *
   * <p>The timer only measures how long the hood has appeared stopped or stalled. It should be reset
   * whenever the limit sensor is tripped, the hood starts moving again, or zeroing is interrupted.
   */
  @Override
  public void resetHoodTimer() {
    stoppedTimer.stop();
    stoppedTimer.reset();
  }

  /**
   * Returns whether the hood currently appears stalled against the hard stop.
   *
   * <p>The fallback zero detector requires both low velocity and elevated current. Low velocity alone
   * is not enough because the hood moves very slowly during zeroing.
   *
   * @return true when the hood appears stopped under load
   */
  private boolean hasHoodStopped() {
    boolean lowVelocity =
        Math.abs(hoodAngularVelocity.getValueAsDouble())
            < HoodConstants.HOOD_ZEROING_VELOCITY_THRESHOLD;

    boolean highCurrent =
        Math.max(
                Math.abs(hoodStatorCurrent.getValueAsDouble()),
                Math.abs(hoodTorqueCurrent.getValueAsDouble()))
            > HoodConstants.HOOD_ZEROING_CURRENT_THRESHOLD;

    boolean stopped = lowVelocity && highCurrent;

    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/LowVelocity", lowVelocity);
    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/HighCurrent", highCurrent);
    Logger.recordOutput(HOOD_TABLE_KEY + "Zero/Stopped", stopped);

    return stopped;
  }

  /**
   * Converts the raw DIO value into the readable sensor state.
   *
   * <p>The current hood zero sensor is active low, so the raw DIO value is compared against
   * {@link HoodConstants#DIO_LIMIT_TRIPPED}. The rest of the hood code should use this method instead
   * of reading the DIO directly.
   *
   * @return true when the bottom limit sensor is active
   */
  private boolean isBottomLimitSensorTripped() {
    return bottomLimitSensor.get() == HoodConstants.DIO_LIMIT_TRIPPED;
  }

  /**
   * Updates the stored supply current limit and applies the full current limit configuration.
   *
   * <p>The stator limit is reapplied with the supply limit so both values stay explicit.
   *
   * @param supplyLimitAmps supply current limit in amps
   */
  @Override
  public void setSupplyCurrentLimit(double supplyLimitAmps) {
    appliedSupplyCurrentLimit = supplyLimitAmps;
    applyCurrentLimits();
  }

  /**
   * Updates the stored stator current limit and applies the full current limit configuration.
   *
   * <p>The supply limit is reapplied with the stator limit so both values stay explicit.
   *
   * @param statorLimitAmps stator current limit in amps
   */
  @Override
  public void setStatorCurrentLimit(double statorLimitAmps) {
    appliedStatorCurrentLimit = statorLimitAmps;
    applyCurrentLimits();
  }

  /**
   * Applies the currently stored supply and stator current limits to the motor controller.
   *
   * <p>Both limits are applied together so changing one limit does not accidentally lose track of the
   * other configured limit.
   */
  private void applyCurrentLimits() {
    var currentLimits = new CurrentLimitsConfigs();

    currentLimits.SupplyCurrentLimit = appliedSupplyCurrentLimit;
    currentLimits.StatorCurrentLimit = appliedStatorCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimitEnable = true;

    var status = hoodMotor.getConfigurator().apply(currentLimits);

    Logger.recordOutput(HOOD_TABLE_KEY + "CurrentLimits/ApplyStatus", status.toString());
    Logger.recordOutput(HOOD_TABLE_KEY + "CurrentLimits/Supply", appliedSupplyCurrentLimit);
    Logger.recordOutput(HOOD_TABLE_KEY + "CurrentLimits/Stator", appliedStatorCurrentLimit);
  }

  /**
   * Applies tunable PID slot values if tuning is enabled and values have changed.
   *
   * <p>This is called from position mode only. The individual slot helpers avoid reapplying TalonFX
   * configs every loop when nothing changed.
   *
   * @param outputs latest requested hood outputs containing tunable gains
   */
  private void applyTunableSlotConfigs(HoodIOOutputs outputs) {
    applyTunableUpSlot(outputs);
    applyTunableDownSlot(outputs);
  }

  /**
   * Applies changed upward-motion PID values to slot 0.
   *
   * @param outputs latest requested hood outputs containing tunable gains
   */
  private void applyTunableUpSlot(HoodIOOutputs outputs) {
    boolean changed =
        slot0.kP != outputs.upKP
            || slot0.kI != outputs.upKI
            || slot0.kD != outputs.upKD
            || slot0.kS != outputs.upKS;

    if (!changed) {
      return;
    }

    slot0.kP = outputs.upKP;
    slot0.kI = outputs.upKI;
    slot0.kD = outputs.upKD;
    slot0.kS = outputs.upKS;

    var status = hoodMotor.getConfigurator().apply(slot0);
    slot0AppliedCount++;

    Logger.recordOutput(HOOD_TABLE_KEY + "Tuning/UpSlotApplyStatus", status.toString());
    Logger.recordOutput(HOOD_TABLE_KEY + "Tuning/UpSlotAppliedCount", slot0AppliedCount);
  }

  /**
   * Applies changed downward-motion PID values to slot 1.
   *
   * @param outputs latest requested hood outputs containing tunable gains
   */
  private void applyTunableDownSlot(HoodIOOutputs outputs) {
    boolean changed =
        slot1.kP != outputs.downKP
            || slot1.kI != outputs.downKI
            || slot1.kD != outputs.downKD
            || slot1.kS != outputs.downKS;

    if (!changed) {
      return;
    }

    slot1.kP = outputs.downKP;
    slot1.kI = outputs.downKI;
    slot1.kD = outputs.downKD;
    slot1.kS = outputs.downKS;

    var status = hoodMotor.getConfigurator().apply(slot1);
    slot1AppliedCount++;

    Logger.recordOutput(HOOD_TABLE_KEY + "Tuning/DownSlotApplyStatus", status.toString());
    Logger.recordOutput(HOOD_TABLE_KEY + "Tuning/DownSlotAppliedCount", slot1AppliedCount);
  }
}