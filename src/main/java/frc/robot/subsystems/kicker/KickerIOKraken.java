package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class KickerIOKraken implements KickerIO {
  private final TalonFX kickerMotor = new TalonFX(KickerConstants.KICKER_CAN_ID);
  private TalonFX secondKickerMotor;
  private boolean twoKickerMotors = true;

  public KickerIOKraken() {
    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.CurrentLimits.SupplyCurrentLimit = KickerConstants.KICKER_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));
    kickerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
    if (twoKickerMotors == true) {
      secondKickerMotor = new TalonFX(KickerConstants.KICKER_2_CAN_ID);
      PhoenixUtil.tryUntilOk(
          5, () -> secondKickerMotor.getConfigurator().apply(kickerConfig, 0.25));
      secondKickerMotor.setControl(
          new Follower(kickerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerSpeed = kickerMotor.get();
    SmartDashboard.putNumber("Kicker Velocity", kickerMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    kickerMotor.set(outputs.desiredKickerSpeed);
  }
}
