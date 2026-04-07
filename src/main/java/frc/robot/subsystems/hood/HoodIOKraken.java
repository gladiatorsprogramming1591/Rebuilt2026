package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_TABLE_KEY;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class HoodIOKraken implements HoodIO {
  // Smart Dashboard keys
  private final String SD_ANGLE_BEFORE_APPLIED_ZERO = HOOD_TABLE_KEY + "Angle before applied zero";
  private final String SD_STOPPED                   = HOOD_TABLE_KEY + "Stopped";
  private final String SD_STOPPED_OVER_TIME         = HOOD_TABLE_KEY + "Stopped over time";
  private final String SD_STOPPED_OVER_TIME_TIMER   = HOOD_TABLE_KEY + "Stopped over time timer";
  private final String SD_ACCEPTED_ZERO_COUNT       = HOOD_TABLE_KEY + "Accepted zero count";
  private final String SD_REJECTED_ZERO_COUNT       = HOOD_TABLE_KEY + "Rejected zero count";
  private final String SD_APPLIED_SUPPLY_CURRENT    = HOOD_TABLE_KEY + "Supply current limit";
  private final String SD_APPLIED_STATOR_CURRENT    = HOOD_TABLE_KEY + "Stator current limit";
  private final String SD_LIMIT_SENSOR_INVERTED    = HOOD_TABLE_KEY + "limit sensor inverted";
  
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
  private final DigitalInput bottomLimitSensor = new DigitalInput(HoodConstants.HOOD_DIO_PORT);
  // Debouncer for DIO zero limit (filters out brief trips of sensor)
  private final Debouncer bottomLimitSensorDebouncer = new Debouncer(HoodConstants.LIMIT_SENSOR_DEBOUNCE_TIME, DebounceType.kRising);
  private final boolean isLimitSensorOutputInverted = bottomLimitSensor instanceof DigitalInput ? true : false;
  private final Timer timer = new Timer();
  private final Slot0Configs slot0 = new Slot0Configs();
  private final PositionTorqueCurrentFOC positionTorqueControl =
  new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  
  // Status Signals
  private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();
  
  private double angleBeforeAppliedZero = 0.0;
  private int acceptedZeroCounter = 0;
  private int rejectedZeroCounter = 0;

  public HoodIOKraken() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_REDUCTION;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));

    slot0.kP = HoodConstants.kP;
    slot0.kD = HoodConstants.kD;
    slot0.kS = HoodConstants.kS;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hoodMotor.getConfigurator().apply(slot0);

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

    SmartDashboard.putNumber(SD_ANGLE_BEFORE_APPLIED_ZERO, 0);
    SmartDashboard.putBoolean(SD_STOPPED, true);
    SmartDashboard.putBoolean(SD_STOPPED_OVER_TIME, true);
    SmartDashboard.putNumber(SD_STOPPED_OVER_TIME_TIMER, 0);
    SmartDashboard.putNumber(SD_ACCEPTED_ZERO_COUNT, acceptedZeroCounter);
    SmartDashboard.putNumber(SD_REJECTED_ZERO_COUNT, rejectedZeroCounter);
    SmartDashboard.putNumber(SD_APPLIED_SUPPLY_CURRENT, HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT);
    SmartDashboard.putNumber(SD_APPLIED_STATOR_CURRENT, HoodConstants.HOOD_STATOR_CURRENT_LIMIT);
    SmartDashboard.putBoolean(SD_LIMIT_SENSOR_INVERTED, isLimitSensorOutputInverted);
  }

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
    inputs.hoodLimitSet = bottomLimitSensor.get();
  }

  @Override
  public void zeroHood() {
    double hoodPosition = hoodAngle.getValueAsDouble();
    angleBeforeAppliedZero = hoodPosition;
    if (!isHoodWithinZeroTolerance().getAsBoolean())
    {
      hoodMotor.setPosition(0.0);
      SmartDashboard.putNumber(SD_ACCEPTED_ZERO_COUNT, ++acceptedZeroCounter);
    } else
    {
      SmartDashboard.putNumber(SD_REJECTED_ZERO_COUNT, ++rejectedZeroCounter);
    }
    SmartDashboard.putNumber(SD_ANGLE_BEFORE_APPLIED_ZERO, angleBeforeAppliedZero);
  }

  @Override
  public BooleanSupplier isHoodWithinZeroTolerance()
  {
    double hoodPosition = hoodAngle.getValueAsDouble();
    return () -> Math.abs(hoodPosition) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  @Override
  public void runHoodToZero() {
    hoodMotor.set(HoodConstants.HOOD_ZEROING_SPEED);
  }

  @Override
  public boolean isHoodAtTrueZero() {
    return bottomLimitSensorDebouncer.calculate(isBottomLimitSensorTripped().getAsBoolean()) && hasHoodStopped()
        || hasHoodStoppedOverTime(HoodConstants.MIN_STATIONARY_DURATION);
  }

  @Override
  public void setSupplyCurrentLimit(double supplyLimitAmps) {
    hoodMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(supplyLimitAmps));
    SmartDashboard.putNumber(SD_APPLIED_SUPPLY_CURRENT, supplyLimitAmps);
  }

  @Override
  public void setStatorCurrentLimit(double statorLimitAmps) {
    hoodMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(statorLimitAmps));
    SmartDashboard.putNumber(SD_APPLIED_STATOR_CURRENT, statorLimitAmps);
  }

  @Override
  public boolean hasHoodStoppedOverTime(double minStationaryDuration) {
    SmartDashboard.putNumber(SD_STOPPED_OVER_TIME_TIMER, timer.get());
    timer.start();
    if (timer.get() < minStationaryDuration) // In seconds
    {
      if (!hasHoodStopped()) {
        timer.reset();
        SmartDashboard.putBoolean(SD_STOPPED_OVER_TIME, false);
        return false;
      }
    } else {
      if (hasHoodStopped()) {
        resetHoodTimer();
        SmartDashboard.putBoolean(SD_STOPPED_OVER_TIME, true);
        return true;
      } else {
        // Corner case for if hood moves right after timer expires
        timer.reset();
        SmartDashboard.putBoolean(SD_STOPPED_OVER_TIME, false);
        return false;
      }
    }
    SmartDashboard.putBoolean(SD_STOPPED_OVER_TIME, false);
    return false;
  }

  // private boolean hasHoodStopped() {
  //   boolean retVal =
  //       hoodSupplyCurrent.getValueAsDouble() > HoodConstants.HOOD_ZEROING_CURRENT_THRESHOLD;
  //   SmartDashboard.putBoolean(SD_STOPPED, retVal);
  //   return retVal;
  // }
  
  private boolean hasHoodStopped() {
    boolean retVal =
        hoodAngularVelocity.getValueAsDouble() < HoodConstants.HOOD_ZEROING_VELOCITY_THRESHOLD;
    SmartDashboard.putBoolean(SD_STOPPED, retVal);
    return retVal;
  }

  @Override
  public void resetHoodTimer() {
    timer.stop();
    timer.reset();
  }

  /**
   * Inverts limit sensor output only if it's a {@code DigitalInput} (i.e. DIO).
   * 
   * @return Boolean supplier of whether the bottom limit sensor has been tripped.
   * @see edu.wpi.first.wpilibj.DigitalInput DigitalInput
   */
  private BooleanSupplier isBottomLimitSensorTripped()
  {
    if (isLimitSensorOutputInverted)
    {
      return () -> !bottomLimitSensor.get();
    } else
    {
      return () -> bottomLimitSensor.get();
    }
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    SmartDashboard.putNumber("Desired Hood Angle", outputs.desiredHoodAngle);
    if (outputs.mode == HoodMode.POSITION) {
      if (Constants.tuningMode) {
        slot0.kP = outputs.kP;
        slot0.kD = outputs.kD;
        slot0.kS = outputs.kS;
        // hoodMotor.getConfigurator().apply(hoodSlot0);
      }

      hoodMotor.setControl(
          positionTorqueControl
              .withPosition(outputs.desiredHoodAngle)
              .withSlot(0)
              .withFeedForward(outputs.kS));
    } else if (outputs.mode == HoodMode.SPEED) {
      hoodMotor.set(outputs.desiredHoodSpeed);
      SmartDashboard.putNumber("Desired Hood Speed", outputs.desiredHoodSpeed);
    }
  }
}
