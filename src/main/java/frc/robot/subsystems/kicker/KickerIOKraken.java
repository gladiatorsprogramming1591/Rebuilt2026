package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.KICKER_TABLE_KEY;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Real kicker IO implementation using one primary Kraken/TalonFX and an optional follower motor.
 *
 * <p>The subsystem commands only the primary motor. When enabled, the secondary kicker motor
 * follows the primary motor through CTRE follower control.
 */
public class KickerIOKraken implements KickerIO {
  private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_CAN_ID);
  private final TalonFX secondKickerMotor =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? new TalonFX(KickerConstants.KICKER_2_CAN_ID) : null;

  private final StatusSignal<AngularVelocity> primaryVelocity = kickerMotor.getVelocity();
  private final StatusSignal<Voltage> primaryAppliedVolts = kickerMotor.getMotorVoltage();
  private final StatusSignal<Current> primarySupplyCurrent = kickerMotor.getSupplyCurrent();
  private final StatusSignal<Current> primaryStatorCurrent = kickerMotor.getStatorCurrent();
  private final StatusSignal<Current> primaryTorqueCurrent = kickerMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> primaryTemperature = kickerMotor.getDeviceTemp();

  private final StatusSignal<AngularVelocity> secondaryVelocity =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getVelocity() : null;
  private final StatusSignal<Voltage> secondaryAppliedVolts =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getMotorVoltage() : null;
  private final StatusSignal<Current> secondarySupplyCurrent =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getSupplyCurrent() : null;
  private final StatusSignal<Current> secondaryStatorCurrent =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getStatorCurrent() : null;
  private final StatusSignal<Current> secondaryTorqueCurrent =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getTorqueCurrent() : null;
  private final StatusSignal<Temperature> secondaryTemperature =
      KickerConstants.HAS_SECOND_KICKER_MOTOR ? secondKickerMotor.getDeviceTemp() : null;

  /**
   * Creates the real kicker IO layer and configures the primary and optional secondary motor.
   *
   * <p>This constructor only configures hardware. The subsystem still controls when the kicker
   * runs.
   */
  public KickerIOKraken() {
    configurePrimaryMotor();

    if (KickerConstants.HAS_SECOND_KICKER_MOTOR) {
      configureSecondaryMotor();
    }

    configureStatusSignals();
  }

  /** Applies base configuration to the primary kicker motor. */
  private void configurePrimaryMotor() {
    var kickerConfig = createKickerConfig();

    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));

    kickerMotor
        .getConfigurator()
        .apply(
            new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(KickerConstants.KICKER_DUTY_CYCLE_RAMP_PERIOD));
  }

  /** Applies base configuration to the secondary kicker motor and sets it to follow the primary. */
  private void configureSecondaryMotor() {
    var kickerConfig = createKickerConfig();

    PhoenixUtil.tryUntilOk(5, () -> secondKickerMotor.getConfigurator().apply(kickerConfig, 0.25));

    secondKickerMotor
        .getConfigurator()
        .apply(
            new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(KickerConstants.KICKER_DUTY_CYCLE_RAMP_PERIOD));

    secondKickerMotor.setControl(
        new Follower(kickerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  /**
   * Creates the shared TalonFX configuration for kicker motors.
   *
   * @return base kicker motor configuration
   */
  private TalonFXConfiguration createKickerConfig() {
    var kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimit = KickerConstants.KICKER_SUPPLY_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return kickerConfig;
  }

  /** Sets status signal update rates and reduces unnecessary CAN bus traffic. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        KickerConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
        primaryVelocity,
        primaryAppliedVolts,
        primarySupplyCurrent,
        primaryStatorCurrent,
        primaryTorqueCurrent,
        primaryTemperature);

    if (KickerConstants.HAS_SECOND_KICKER_MOTOR) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          KickerConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
          secondaryVelocity,
          secondaryAppliedVolts,
          secondarySupplyCurrent,
          secondaryStatorCurrent,
          secondaryTorqueCurrent,
          secondaryTemperature);
    }

    kickerMotor.optimizeBusUtilization();

    if (KickerConstants.HAS_SECOND_KICKER_MOTOR) {
      secondKickerMotor.optimizeBusUtilization();
    }
  }

  /**
   * Refreshes kicker motor status signals.
   *
   * @param inputs container updated with the latest kicker hardware state
   */
  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        primaryVelocity,
        primaryAppliedVolts,
        primarySupplyCurrent,
        primaryStatorCurrent,
        primaryTorqueCurrent,
        primaryTemperature);

    inputs.primaryConnected = kickerMotor.isConnected();
    inputs.primaryDutyCycle = kickerMotor.get();
    inputs.primaryVelocity = primaryVelocity.getValueAsDouble();
    inputs.primaryAppliedVolts = primaryAppliedVolts.getValueAsDouble();
    inputs.primarySupplyCurrent = primarySupplyCurrent.getValueAsDouble();
    inputs.primaryStatorCurrent = primaryStatorCurrent.getValueAsDouble();
    inputs.primaryTorqueCurrent = primaryTorqueCurrent.getValueAsDouble();
    inputs.primaryTemperature = primaryTemperature.getValueAsDouble();

    if (KickerConstants.HAS_SECOND_KICKER_MOTOR) {
      BaseStatusSignal.refreshAll(
          secondaryVelocity,
          secondaryAppliedVolts,
          secondarySupplyCurrent,
          secondaryStatorCurrent,
          secondaryTorqueCurrent,
          secondaryTemperature);

      inputs.secondaryConnected = secondKickerMotor.isConnected();
      inputs.secondaryDutyCycle = secondKickerMotor.get();
      inputs.secondaryVelocity = secondaryVelocity.getValueAsDouble();
      inputs.secondaryAppliedVolts = secondaryAppliedVolts.getValueAsDouble();
      inputs.secondarySupplyCurrent = secondarySupplyCurrent.getValueAsDouble();
      inputs.secondaryStatorCurrent = secondaryStatorCurrent.getValueAsDouble();
      inputs.secondaryTorqueCurrent = secondaryTorqueCurrent.getValueAsDouble();
      inputs.secondaryTemperature = secondaryTemperature.getValueAsDouble();
    } else {
      inputs.secondaryConnected = false;
      inputs.secondaryDutyCycle = 0.0;
      inputs.secondaryVelocity = 0.0;
      inputs.secondaryAppliedVolts = 0.0;
      inputs.secondarySupplyCurrent = 0.0;
      inputs.secondaryStatorCurrent = 0.0;
      inputs.secondaryTorqueCurrent = 0.0;
      inputs.secondaryTemperature = 0.0;
    }
  }

  /**
   * Applies the requested kicker motor output.
   *
   * <p>The secondary motor follows the primary motor, so only the primary motor is directly
   * commanded.
   *
   * @param outputs latest requested kicker outputs from the subsystem
   */
  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    double speed = MathUtil.clamp(outputs.desiredKickerSpeed, -1.0, 1.0);
    kickerMotor.set(speed);

    Logger.recordOutput(KICKER_TABLE_KEY + "AppliedSpeed", speed);
  }
}
