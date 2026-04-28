package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_TABLE_KEY;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Real hopper IO implementation using one Kraken/TalonFX belt motor and a CANrange empty sensor.
 *
 * <p>The hopper empty sensor is treated as optional. If it is disconnected, the hopper reports not
 * empty so shooting logic does not stop early because of missing sensor data.
 */
public class HopperIOKraken implements HopperIO {
  private final TalonFX beltMotor = new TalonFX(HopperConstants.BELT_CAN_ID);
  private final CANrange hopperEmptySensor =
      new CANrange(HopperConstants.HOPPER_EMPTY_CANRANGE_CAN_ID);

  private final StatusSignal<Current> beltCurrent = beltMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> beltVelocity = beltMotor.getVelocity();
  private final StatusSignal<Distance> hopperEmptyDistance = hopperEmptySensor.getDistance();

  private boolean usingIntakeCurrentLimit = false;
  private double appliedSupplyCurrentLimit = HopperConstants.BELT_CURRENT_LIMIT;

  /**
   * Creates the real hopper IO layer and configures the belt motor and status signals.
   *
   * <p>The subsystem owns when the belt runs. This constructor only configures the hardware.
   */
  public HopperIOKraken() {
    configureBeltMotor();
    configureStatusSignals();
  }

  /**
   * Applies the base TalonFX configuration for the hopper belt motor.
   *
   * <p>The belt motor is coasted when not powered so it does not fight fuel movement.
   */
  private void configureBeltMotor() {
    var beltConfig = new TalonFXConfiguration();

    beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    beltConfig.CurrentLimits.SupplyCurrentLimit = appliedSupplyCurrentLimit;
    beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    PhoenixUtil.tryUntilOk(5, () -> beltMotor.getConfigurator().apply(beltConfig, 0.25));
  }

  /** Sets status signal update rates and reduces unnecessary CAN bus traffic. */
  private void configureStatusSignals() {
    BaseStatusSignal.setUpdateFrequencyForAll(50, beltCurrent, beltVelocity, hopperEmptyDistance);

    beltMotor.optimizeBusUtilization();
    hopperEmptySensor.optimizeBusUtilization();
  }

  /**
   * Refreshes belt motor signals and reads the hopper empty CANrange.
   *
   * <p>The CANrange must be connected before its distance is trusted. If it is disconnected, hopper
   * empty is forced false.
   *
   * @param inputs container updated with the latest hopper hardware state
   */
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(beltCurrent, beltVelocity, hopperEmptyDistance);

    inputs.beltCurrent = beltCurrent.getValueAsDouble();
    inputs.beltVelocity = beltVelocity.getValueAsDouble();

    inputs.hopperEmptySensorConnected = isHopperEmptySensorConnected();
    inputs.hopperEmptyDistance =
        inputs.hopperEmptySensorConnected ? hopperEmptyDistance.getValueAsDouble() : 1.0;
    inputs.hopperEmpty =
        inputs.hopperEmptySensorConnected && isHopperEmptyDistance(inputs.hopperEmptyDistance);

    Logger.recordOutput(
        HOPPER_TABLE_KEY + "Sensor/EmptyThreshold",
        HopperConstants.HOPPER_EMPTY_DISTANCE_LIMIT.getAsDouble());
  }

  /**
   * Applies the requested hopper output to the belt motor.
   *
   * <p>Current limit mode is applied before motor output so the requested current limit is active
   * when the belt starts running.
   *
   * @param outputs latest requested hopper outputs from the subsystem
   */
  @Override
  public void applyOutputs(HopperIOOutputs outputs) {
    applyCurrentLimitMode(outputs.useBeltWhileIntakeCurrent);
    applyBeltSpeed(outputs.beltSpeed);
  }

  /**
   * Applies open-loop belt motor output.
   *
   * <p>The output is clamped as a final safety check before being sent to the motor controller.
   *
   * @param speed requested belt percent output
   */
  private void applyBeltSpeed(double speed) {
    beltMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  /**
   * Applies the requested current limit mode if it changed.
   *
   * <p>The lower current limit is used for gentle belt agitation while intaking. Reverse and normal
   * belt commands use the full current limit.
   *
   * @param useIntakeCurrentLimit true to use the lower intaking/agitation current limit
   */
  private void applyCurrentLimitMode(boolean useIntakeCurrentLimit) {
    if (usingIntakeCurrentLimit == useIntakeCurrentLimit) {
      return;
    }

    usingIntakeCurrentLimit = useIntakeCurrentLimit;

    double requestedCurrentLimit =
        usingIntakeCurrentLimit
            ? HopperConstants.BELT_INTAKE_CURRENT_LIMIT
            : HopperConstants.BELT_CURRENT_LIMIT;

    applySupplyCurrentLimit(requestedCurrentLimit);
  }

  /**
   * Returns whether the CANrange hopper empty sensor is connected.
   *
   * @return true when the CANrange is connected on CAN
   */
  private boolean isHopperEmptySensorConnected() {
    return hopperEmptySensor.isConnected();
  }

  /**
   * Returns whether a connected CANrange distance should be interpreted as hopper empty.
   *
   * @param distanceMeters CANrange distance in meters
   * @return true when the measured distance is greater than the configured empty threshold
   */
  private boolean isHopperEmptyDistance(double distanceMeters) {
    return distanceMeters >= HopperConstants.HOPPER_EMPTY_DISTANCE_LIMIT.getAsDouble();
  }

  /**
   * Applies a supply current limit to the belt motor.
   *
   * @param supplyLimitAmps supply current limit in amps
   */
  private void applySupplyCurrentLimit(double supplyLimitAmps) {
    appliedSupplyCurrentLimit = supplyLimitAmps;

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(appliedSupplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true);

    var status = beltMotor.getConfigurator().apply(currentLimits);

    Logger.recordOutput(HOPPER_TABLE_KEY + "CurrentLimit/ApplyStatus", status.toString());
    Logger.recordOutput(HOPPER_TABLE_KEY + "CurrentLimit/Supply", appliedSupplyCurrentLimit);
    Logger.recordOutput(
        HOPPER_TABLE_KEY + "CurrentLimit/UsingIntakeLimit", usingIntakeCurrentLimit);
  }
}
