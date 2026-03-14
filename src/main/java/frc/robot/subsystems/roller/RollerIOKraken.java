package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class RollerIOKraken implements RollerIO {
  private final TalonFX rollerTopMotor = new TalonFX(RollerConstants.ROLLER_TOP_CAN_ID);
  private final TalonFX rollerBottomMotor = new TalonFX(RollerConstants.ROLLER_BOTTOM_CAN_ID);
  TalonFXConfiguration rollerTopConfig = new TalonFXConfiguration();
  TalonFXConfiguration rollerBottomConfig = new TalonFXConfiguration();

  public RollerIOKraken() {
    rollerTopConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_CURRENT_LIMIT;
    rollerTopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> rollerTopMotor.getConfigurator().apply(rollerTopConfig, 0.25));

    rollerBottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerBottomConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_CURRENT_LIMIT;
    rollerBottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerBottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(
        5, () -> rollerBottomMotor.getConfigurator().apply(rollerBottomConfig, 0.25));
    rollerTopMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    SmartDashboard.putNumber(
        "Top Roller Velocity", rollerTopMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Bottom Roller Velocity", rollerBottomMotor.getVelocity().getValueAsDouble());
    inputs.bottomRollerCurrent = rollerBottomMotor.getSupplyCurrent().getValueAsDouble();
    inputs.topRollerCurrent = rollerTopMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    rollerTopMotor.set(outputs.topRollerSpeed);
    rollerBottomMotor.set(outputs.bottomRollerSpeed);
    // Swap current limits only once when requested currents change
    if(!outputs.usingLowerCurrent && outputs.useRollerWhileIntakeCurrent) {
      rollerBottomMotor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(RollerConstants.BOTTOM_ROLLER_INTAKE_CURRENT_LIMIT));
      outputs.usingLowerCurrent = true;
    } else if (outputs.usingLowerCurrent && !outputs.useRollerWhileIntakeCurrent) {
      rollerBottomMotor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT));
      outputs.usingLowerCurrent = false;
    }
  }
}
