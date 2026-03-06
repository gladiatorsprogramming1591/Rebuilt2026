package frc.robot.subsystems.hood;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class HoodIOKraken implements HoodIO {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
  private final DigitalInput hoodLimit = new DigitalInput(HoodConstants.HOOD_DIO_PORT);
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

      PhoenixUtil.registerSignals(
          false,
          hoodAppliedVolts,
          hoodAngle,
          hoodAngularVelocity,
          hoodTorqueCurrent,
          hoodSupplyCurrent,
          hoodTemperature);

    }
  
    @Override
    public void setHoodSpeed(AngularVelocity angularVelocity) {
      this.angularVelocity = MathUtil.clamp(this.angularVelocity, -HoodConstants.HOOD_MAX_SPEED, HoodConstants.HOOD_MAX_SPEED);
      hoodMotor.set(this.angularVelocity);
      SmartDashboard.putNumber("Hood Speed", this.angularVelocity);
    }
  
    @Override
    public void setHoodPosition(double angle) {
      this.angle = MathUtil.clamp(angle, HoodConstants.HOOD_LOWER_LIMIT, HoodConstants.HOOD_UPPER_LIMIT);
<<<<<<< HEAD
      SmartDashboard.putNumber("Hood Angle", hoodAngle.getValueAsDouble());
=======
      SmartDashboard.putNumber("Hood Angle", this.angle);
>>>>>>> origin/KileyHood
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
<<<<<<< HEAD
    inputs.hoodSpeed = hoodAngularVelocity.getValueAsDouble();
    inputs.hoodAngle = hoodAngle.getValueAsDouble();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodTorqueCurrent = hoodTorqueCurrent.getValueAsDouble();
    inputs.hoodTemperature = hoodTemperature.getValueAsDouble();
  }

  @Override 
  public void applyOutputs(HoodIOOutputs outputs) {
    setHoodSpeed(outputs.hoodSpeed);
    setHoodPosition(outputs.hoodAngle);
=======
    inputs.hoodSpeed = this.speed;
    inputs.hoodAngle = this.angle; 
>>>>>>> origin/KileyHood
  }
}
