package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerIOKraken implements KickerIO {
  private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_CAN_ID);
  private double speed = 0.0;

  public KickerIOKraken() {
    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = KickerConstants.KICKER_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kickerMotor.getConfigurator().apply(kickerConfig, 0.25);
    kickerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void setKickerSpeed(double speed) {
    this.speed = speed;
    kickerMotor.set(this.speed);
    SmartDashboard.putNumber("Kicker Speed", this.speed);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerSpeed = this.speed;
    SmartDashboard.putNumber("Kicker Velocity", kickerMotor.getVelocity().getValueAsDouble());
  }
}
