package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
    private final SparkBase intakeMotor; 

    public Intake() {
        intakeMotor = new SparkFlex(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless); 
    }

    public void setSpeed(CommandXboxController controller) {
        double speed = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        intakeMotor.set(speed);
        SmartDashboard.putNumber("Intake Speed", speed);
    }
}
