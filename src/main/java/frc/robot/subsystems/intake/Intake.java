package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_CAN_ID, TunerConstants.kCANBus);
  }

  public void setSpeed(CommandXboxController controller) {
    double speed = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    intakeMotor.set(speed);
    SmartDashboard.putNumber("Intake Speed", speed);
  }
}
