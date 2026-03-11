package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class RollerIOKraken implements RollerIO {
  private final TalonFX rollerTopMotor = new TalonFX(RollerConstants.ROLLER_TOP_CAN_ID);
  private final TalonFX rollerBottomMotor = new TalonFX(RollerConstants.ROLLER_BOTTOM_CAN_ID);
  private double speed = 0.0;

  public RollerIOKraken() {
    var rollerTopConfig = new TalonFXConfiguration();
    rollerTopConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_CURRENT_LIMIT;
    rollerTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PhoenixUtil.tryUntilOk(5, () -> rollerTopMotor.getConfigurator().apply(rollerTopConfig, 0.25));

    var rollerBottomConfig = new TalonFXConfiguration();
    rollerBottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerBottomConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_CURRENT_LIMIT;
    rollerBottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerBottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PhoenixUtil.tryUntilOk(
        5, () -> rollerBottomMotor.getConfigurator().apply(rollerBottomConfig, 0.25));
    rollerTopMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void setTopRollerSpeed(double speed) {
    this.speed = speed;
    rollerTopMotor.set(this.speed);
    SmartDashboard.putNumber("Top Roller Speed", this.speed);
  }

  @Override
  public void setBottomRollerSpeed(double speed) {
    this.speed = speed;
    rollerBottomMotor.set(this.speed);
    SmartDashboard.putNumber("Bottom Roller Speed", this.speed);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerSpeed = this.speed;
    SmartDashboard.putNumber(
        "Top Roller Velocity", rollerTopMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Bottom Roller Velocity", rollerBottomMotor.getVelocity().getValueAsDouble());
  }
}
