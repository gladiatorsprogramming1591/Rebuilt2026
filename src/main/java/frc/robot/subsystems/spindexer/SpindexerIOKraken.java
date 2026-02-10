package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerIOKraken implements SpindexerIO {
  private final TalonFX spindexerMotor;
  private double speed = 0.0;

  public SpindexerIOKraken() {
    // intakeMotor = new SparkFlex(SpindexerConstants.SPINDEXER_CAN_ID, MotorType.kBrushless);
    spindexerMotor = new TalonFX(SpindexerConstants.SPINDEXER_CAN_ID);
    spindexerMotor
        .getConfigurator()
        .apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(5.0));
  }

  public void setSpeed(double speed) {
    spindexerMotor.set(speed);
    this.speed = speed;
    SmartDashboard.putNumber("Spindexer Speed", speed);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.speed = this.speed;
  }
}
