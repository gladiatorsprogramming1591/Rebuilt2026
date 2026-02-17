package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerIOKraken implements RollerIO {
  private final TalonFX rollerMotor = new TalonFX(RollerConstants.ROLLER_CAN_ID);
  private double speed = 0.0;

  public RollerIOKraken() {
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
  public void setRollerSpeed(double speed) {
    this.speed = MathUtil.clamp(speed, 0, 1.0);
    rollerMotor.set(this.speed);
    SmartDashboard.putNumber("Roller Speed", this.speed);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerSpeed = this.speed;
  }
}
