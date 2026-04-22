package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;

public class HopperIOKraken implements HopperIO {
  // private final TalonFX rollerTopMotor = new TalonFX(RollerConstants.ROLLER_TOP_CAN_ID);
  private final TalonFX beltMotor = new TalonFX(HopperConstants.BELT_CAN_ID);
  TalonFXConfiguration beltConfig = new TalonFXConfiguration();
  CANrange hopperEmptySensor = new CANrange(HopperConstants.HOPPER_EMPTY_CANRANGE_CAN_ID);
  StatusSignal<Distance> hopperEmptyDistance;

  public HopperIOKraken() {
    beltConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    beltConfig.CurrentLimits.SupplyCurrentLimit = HopperConstants.BELT_CURRENT_LIMIT;
    beltConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> beltMotor.getConfigurator().apply(beltConfig, 0.25));
    hopperEmptyDistance = hopperEmptySensor.getDistance();
    // rollerTopMotor
    //     .getConfigurator()
    //     .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    SmartDashboard.putNumber("Belt Velocity", beltMotor.getVelocity().getValueAsDouble());
    inputs.beltCurrent = beltMotor.getSupplyCurrent().getValueAsDouble();
    BaseStatusSignal.refreshAll(hopperEmptyDistance);
    inputs.hopperEmptyDistance = hopperEmptyDistance.getValueAsDouble();
    inputs.hopperEmpty =
        inputs.hopperEmptyDistance >= HopperConstants.HOPPER_EMPTY_DISTANCE_LIMIT.getAsDouble();
  }

  @Override
  public void applyOutputs(HopperIOOutputs outputs) {
    beltMotor.set(outputs.beltSpeed);
    // Swap current limits only once when requested currents change
    if (!outputs.usingLowerCurrent && outputs.useBeltWhileIntakeCurrent) {
      beltMotor
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(HopperConstants.BELT_INTAKE_CURRENT_LIMIT));
      outputs.usingLowerCurrent = true;
    } else if (outputs.usingLowerCurrent && !outputs.useBeltWhileIntakeCurrent) {
      beltMotor
          .getConfigurator()
          .apply(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(HopperConstants.BELT_CURRENT_LIMIT));
      outputs.usingLowerCurrent = false;
    }
  }
}
