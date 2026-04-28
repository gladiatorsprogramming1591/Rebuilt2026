package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.HOOD_TABLE_KEY;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
  private final Slot1Configs slot1 = new Slot1Configs();
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
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_REDUCTION;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));

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

  private void applyTunableSlotConfigs(HoodIOOutputs outputs) {
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot0 Up kP Output", outputs.upKP);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot0 Up kI Output", outputs.upKI);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot0 Up kD Output", outputs.upKD);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot0 Up kS Output", outputs.upKS);

  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot1 Down kP Output", outputs.downKP);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot1 Down kI Output", outputs.downKI);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot1 Down kD Output", outputs.downKD);
  SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot1 Down kS Output", outputs.downKS);

  boolean slot0Changed =
      slot0.kP != outputs.upKP
          || slot0.kI != outputs.upKI
          || slot0.kD != outputs.upKD
          || slot0.kS != outputs.upKS;

  if (slot0Changed) {
    slot0.kP = outputs.upKP;
    slot0.kI = outputs.upKI;
    slot0.kD = outputs.upKD;
    slot0.kS = outputs.upKS;

    var status = hoodMotor.getConfigurator().apply(slot0);
    SmartDashboard.putString(HOOD_TABLE_KEY + "Slot0 Apply Status", status.toString());
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot0 Applied Count",
        SmartDashboard.getNumber(HOOD_TABLE_KEY + "Slot0 Applied Count", 0) + 1);
  }

  boolean slot1Changed =
      slot1.kP != outputs.downKP
          || slot1.kI != outputs.downKI
          || slot1.kD != outputs.downKD
          || slot1.kS != outputs.downKS;

  if (slot1Changed) {
    slot1.kP = outputs.downKP;
    slot1.kI = outputs.downKI;
    slot1.kD = outputs.downKD;
    slot1.kS = outputs.downKS;

    var status = hoodMotor.getConfigurator().apply(slot1);
    SmartDashboard.putString(HOOD_TABLE_KEY + "Slot1 Apply Status", status.toString());
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Slot1 Applied Count",
        SmartDashboard.getNumber(HOOD_TABLE_KEY + "Slot1 Applied Count", 0) + 1);
  }
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
    hoodMotor.setPosition(0.0);
    SmartDashboard.putNumber(SD_ACCEPTED_ZERO_COUNT, ++acceptedZeroCounter);
    SmartDashboard.putNumber(SD_ANGLE_BEFORE_APPLIED_ZERO, angleBeforeAppliedZero);
  }

  @Override
  public BooleanSupplier isHoodWithinZeroTolerance()
  {
    return () -> Math.abs(hoodAngle.getValueAsDouble()) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  @Override
  public void runHoodToZero() {
    hoodMotor.set(HoodConstants.HOOD_ZEROING_SPEED);
  }

  @Override
public boolean isHoodAtTrueZero() {
  boolean zeroSensorTripped =
      bottomLimitSensorDebouncer.calculate(isBottomLimitSensorTripped().getAsBoolean());

  if (zeroSensorTripped) {
    resetHoodTimer();
    return true;
  }

  return hasHoodStoppedOverTime(HoodConstants.MIN_STATIONARY_DURATION);
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
      Math.abs(hoodAngularVelocity.getValueAsDouble())
          < HoodConstants.HOOD_ZEROING_VELOCITY_THRESHOLD;
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
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Desired Hood Angle", outputs.desiredHoodAngle);
    SmartDashboard.putNumber(HOOD_TABLE_KEY + "Desired Hood Speed", outputs.desiredHoodSpeed);
    if (outputs.mode == HoodMode.POSITION) {
      if (Constants.tuningMode) {
        applyTunableSlotConfigs(outputs);
      }

      double currentAngle = hoodAngle.getValueAsDouble();
      boolean movingDown = outputs.desiredHoodAngle < currentAngle;
      int selectedSlot = movingDown ? 1 : 0;

      SmartDashboard.putNumber(HOOD_TABLE_KEY + "Selected PID Slot", selectedSlot);
      SmartDashboard.putNumber(HOOD_TABLE_KEY + "Position Current Angle", currentAngle);
      SmartDashboard.putNumber(HOOD_TABLE_KEY + "Position Desired Angle", outputs.desiredHoodAngle);
      SmartDashboard.putBoolean(HOOD_TABLE_KEY + "Position Moving Down", movingDown);

      hoodMotor.setControl(
          positionTorqueControl
              .withPosition(outputs.desiredHoodAngle)
              .withSlot(selectedSlot));
    } else if (outputs.mode == HoodMode.SPEED) {
      hoodMotor.set(outputs.desiredHoodSpeed);
    }
  }
}
