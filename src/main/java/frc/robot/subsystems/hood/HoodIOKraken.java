package frc.robot.subsystems.hood;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
  private final DigitalInput hoodDioLimit = new DigitalInput(HoodConstants.HOOD_DIO_PORT);
  // private final Trigger zeroTrigger;
  // DIO returns true when circuit is open, and false when closed (limit sensor tripped).
  private final boolean dioLimitTripped = false;
  // Debouncer for zero limit (filters out brief trips of limit)
  private final Debouncer limitDebounce = new Debouncer(0.5, DebounceType.kFalling);
  private double positionOffset = 0;
  private int acceptedZeroCounter = 0;
  private int rejectedZeroCounter = 0;
  private final Timer timer = new Timer();
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Slot0Configs hoodSlot0 = new Slot0Configs();

  private final VoltageOut zeroControl = new VoltageOut(-2).withUpdateFreqHz(0.0);

  private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

  public HoodIOKraken() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.HOOD_STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.HOOD_MOTOR_REDUCTION;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));

    hoodSlot0.kP = HoodConstants.kP;
    hoodSlot0.kD = HoodConstants.kD;
    hoodSlot0.kS = HoodConstants.kS;
    hoodSlot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    hoodMotor.getConfigurator().apply(hoodSlot0);

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

    // zeroTrigger = new Trigger(() -> isHoodAtTrueZero());

    SmartDashboard.putNumber("Hood offset rots", 0);
    SmartDashboard.putBoolean("hasHoodStopped", true);
    SmartDashboard.putBoolean("hasHoodStoppedOverTime", true);
    SmartDashboard.putNumber("hasHoodStoppedOverTime timer", 0);
    SmartDashboard.putNumber("Hood accepted zero counter", acceptedZeroCounter);
    SmartDashboard.putNumber("Hood rejected zero counter", rejectedZeroCounter);
    SmartDashboard.putNumber("Hood applied supply current", HoodConstants.HOOD_SUPPLY_CURRENT_LIMIT);
    SmartDashboard.putNumber("Hood applied stator current", HoodConstants.HOOD_STATOR_CURRENT_LIMIT);
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
    inputs.hoodLimitSet = hoodDioLimit.get();
  }

  @Override
  public void zeroHood() {
    double hoodPosition = hoodAngle.getValueAsDouble();
    positionOffset = hoodPosition;
    if (!isHoodWithinZeroTolerance().getAsBoolean())
    {
      hoodMotor.setPosition(0.0);
      SmartDashboard.putNumber("Hood accepted zero counter", ++acceptedZeroCounter);
    } else
    {
      SmartDashboard.putNumber("Hood rejected zero counter", ++rejectedZeroCounter);
    }
    SmartDashboard.putNumber("Hood offset rots", positionOffset);
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
    return limitDebounce.calculate(hoodDioLimit.get()) == dioLimitTripped && hasHoodStopped()
        || hasHoodStoppedOverTime(HoodConstants.MIN_STATIONARY_DURATION);
  }

  @Override
  public void setSupplyCurrentLimit(double supplyLimitAmps) {
    hoodMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(supplyLimitAmps));
    SmartDashboard.putNumber("Hood applied supply current", supplyLimitAmps);
  }

  @Override
  public void setStatorCurrentLimit(double statorLimitAmps) {
    hoodMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(statorLimitAmps));
    SmartDashboard.putNumber("Hood applied stator current", statorLimitAmps);
  }

  @Override
  public boolean hasHoodStoppedOverTime(double minStationaryDuration) {
    SmartDashboard.putNumber("hasHoodStoppedOverTime timer", timer.get());
    timer.start();
    if (timer.get() < minStationaryDuration) // In seconds
    {
      if (!hasHoodStopped()) {
        timer.reset();
        SmartDashboard.putBoolean("hasHoodStoppedOverTime", false);
        return false;
      }
    } else {
      if (hasHoodStopped()) {
        resetHoodTimer();
        SmartDashboard.putBoolean("hasHoodStoppedOverTime", true);
        return true;
      } else {
        // Corner case for if hood moves right after timer expires
        timer.reset();
        SmartDashboard.putBoolean("hasHoodStoppedOverTime", false);
        return false;
      }
    }
    SmartDashboard.putBoolean("hasHoodStoppedOverTime", false);
    return false;
  }

  private boolean hasHoodStopped() {
    boolean retVal =
        hoodSupplyCurrent.getValueAsDouble() > HoodConstants.HOOD_ZEROING_CURRENT_THRESHOLD;
    SmartDashboard.putBoolean("hasHoodStopped", retVal);
    return retVal;
  }

  @Override
  public void resetHoodTimer() {
    timer.stop();
    timer.reset();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    SmartDashboard.putNumber("Desired Hood Angle", outputs.desiredHoodAngle);
    if (outputs.mode == HoodMode.POSITION) {
      if (Constants.tuningMode) {
        hoodSlot0.kP = outputs.kP;
        hoodSlot0.kD = outputs.kD;
        hoodSlot0.kS = outputs.kS;
        // hoodMotor.getConfigurator().apply(hoodSlot0);
      }

      hoodMotor.setControl(
          positionControl
              .withPosition(outputs.desiredHoodAngle)
              .withSlot(0)
              .withFeedForward(outputs.kS));
    } else if (outputs.mode == HoodMode.SPEED) {
      hoodMotor.set(outputs.desiredHoodSpeed);
      SmartDashboard.putNumber("Desired Hood Speed", outputs.desiredHoodSpeed);
    }
  }
}
