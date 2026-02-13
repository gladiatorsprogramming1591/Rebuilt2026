package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerIOKraken implements RollerIO {
  private final TalonFX rollerMotor;
  private double speed = 0.0;

  public RollerIOKraken() {
    // intakeMotor = new SparkFlex(RollerConstants.ROLLER_CAN_ID, MotorType.kBrushless);
    rollerMotor = new TalonFX(RollerConstants.ROLLER_CAN_ID);
    rollerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
    this.speed = speed;
    SmartDashboard.putNumber("Roller Speed", speed);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.speed = this.speed;
  }
}
