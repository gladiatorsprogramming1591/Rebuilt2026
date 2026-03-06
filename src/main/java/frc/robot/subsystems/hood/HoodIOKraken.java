package frc.robot.subsystems.hood;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodIOKraken implements HoodIO {
  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_CAN_ID);
<<<<<<< HEAD
  private double angularVelocity = 0.0;

  private final StatusSignal<Angle> hoodAngle = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodAngularVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodTorqueCurrent = hoodMotor.getTorqueCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

=======
  private double speed = 0.0;
  private double angle = 0.0;
  
>>>>>>> origin/KileyHood
    public HoodIOKraken() {
      var hoodConfig = new TalonFXConfiguration();
      hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
      hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      hoodMotor.getConfigurator().apply(hoodConfig, 0.25);
      hoodMotor
          .getConfigurator()
          .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
    }
  
    @Override
<<<<<<< HEAD
    public void setHoodSpeed(AngularVelocity angularVelocity) {
      
      hoodMotor.set(this.angularVelocity);
=======
    public void setHoodSpeed(double speed) {
      this.speed = MathUtil.clamp(speed, 0, 1.0);
      hoodMotor.set(this.speed);
>>>>>>> origin/KileyHood
      SmartDashboard.putNumber("Hood Speed", this.speed);
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
