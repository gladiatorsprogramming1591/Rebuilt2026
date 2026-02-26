package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerIOKraken implements KickerIO {
  private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_CAN_ID);
  private double speed = 0.0;

  public KickerIOKraken() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = KickerConstants.KICKER_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerMotor.getConfigurator().apply(rollerConfig, 0.25);
    kickerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void setKickerSpeed(double speed) {
    this.speed = MathUtil.clamp(speed, 0, 1.0);
    kickerMotor.set(this.speed);
    SmartDashboard.putNumber("Kicker Speed", this.speed);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerSpeed = this.speed;
    SmartDashboard.putNumber("Kicker Velocity", kickerMotor.getVelocity().getValueAsDouble());
  }
}
