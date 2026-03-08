package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class HoodIOKraken implements HoodIO {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
  private final DigitalInput hoodLimit = new DigitalInput(HoodConstants.HOOD_DIO_PORT);
  private final boolean limitTripped =
      false; // DIO returns true when circuit is open, and false when closed (limit sensor tripped).
  private double positionOffset = 0;
  private Timer timer = new Timer();
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Slot0Configs hoodSlot0 = new Slot0Configs();

  private final VoltageOut zeroControl = new VoltageOut(-2).withUpdateFreqHz(0.0);
  // private double angularVelocity = 0.0;
  // private double speed = 0.0;
  // private double angle = 0.0;

  private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

  public HoodIOKraken() {
    var hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
        hoodTorqueCurrent,
        hoodSupplyCurrent,
        hoodTemperature);

    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodAppliedVolts,
        hoodAngle,
        hoodAngularVelocity,
        hoodTorqueCurrent,
        hoodSupplyCurrent,
        hoodTemperature);

    inputs.hoodSpeed = hoodAngularVelocity.getValueAsDouble();
    inputs.hoodAngle = hoodAngle.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodTemperature = hoodTemperature.getValueAsDouble();
    inputs.hoodLimitSet = hoodLimit.get();
  }

  @Override
  public void setHoodSpeed(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void zero() {
    positionOffset = hoodMotor.getPosition().getValueAsDouble() * -1;
    SmartDashboard.putNumber("Hood offset rots", positionOffset);
  }

  @Override
  public void driveToZero() {
    hoodMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(HoodConstants.HOOD_ZEROING_CURRENT_LIMIT));
    hoodMotor.set(HoodConstants.HOOD_ZEROING_SPEED);
  }

  @Override
  public boolean isHoodAtTrueZero() {
    return hoodLimit.get() == limitTripped && isHoodStationary(false) || isHoodStationary(true);
  }

  @Override
  public void setHoodCurrentLimit(double currentLimit) {
    hoodMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimit));
  }

  @Override
  public boolean isHoodStationary(boolean withTimer) {
    if (!withTimer)
      return (Math.abs(hoodMotor.getVelocity().getValueAsDouble())
          < HoodConstants.HOOD_ZEROING_VEL_TOLERANCE);
    timer.start();
    if (timer.get() <= HoodConstants.HOOD_STATIONARY_DELAY) // In seconds
    {
      SmartDashboard.putNumber("isHoodStationary timer", timer.get());
      if (Math.abs(hoodMotor.getVelocity().getValueAsDouble())
          > HoodConstants.HOOD_ZEROING_VEL_TOLERANCE) {
        timer.stop();
        timer.reset();
        SmartDashboard.putBoolean("isHoodStationary", false);
        return false;
      }
    } else {
      timer.stop();
      timer.reset();
      SmartDashboard.putBoolean("isHoodStationary", true);
      return true;
    }
    return false;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodMode.POSITION) {
      hoodMotor.setControl(
          positionControl
              .withPosition(outputs.desiredHoodAngle + positionOffset)
              .withSlot(0)
              .withFeedForward(outputs.kS));
    }
  }
}
