package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeMotor;
  private double speed = 0.0;

  public IntakeIOKraken() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_CAN_ID, TunerConstants.kCANBus);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
    SmartDashboard.putNumber("Intake Speed", speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speed = this.speed;
  }
}
