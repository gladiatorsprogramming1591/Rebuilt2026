package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerIOKraken implements RollerIO {
  private final TalonFX rollerMotor;
  private double volts = 0.0;

  public RollerIOKraken() {
    // intakeMotor = new SparkFlex(RollerConstants.ROLLER_CAN_ID, MotorType.kBrushless);
    rollerMotor = new TalonFX(RollerConstants.ROLLER_CAN_ID);
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.ROLLER_CURRENT_LIMIT;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(rollerConfig, 0.25);
    rollerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  @Override
  public void setRollerSpeed(double volts) {
    this.volts = MathUtil.clamp(volts, 0, 12);
    rollerMotor.setVoltage(this.volts);
    SmartDashboard.putNumber("Roller Applied Volts", this.volts);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerSpeed = this.volts;
  }
}
