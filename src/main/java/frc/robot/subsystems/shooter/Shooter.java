package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final InterpolatingDoubleTreeMap
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command runShooterMotor() {
    return runEnd(
        () -> {
          io.setShooterSpeed(ShooterConstants.SHOOTER_MOTOR_SPEED);
        },
        () -> {
          io.setShooterSpeed(0.0);
        });
  }
}
