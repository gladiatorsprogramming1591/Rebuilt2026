package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOKraken implements IntakeIO {
  private final SparkBase intakeMotor;
  private double speed = 0.0;

  public IntakeIOKraken() {
    intakeMotor = new SparkFlex(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
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
