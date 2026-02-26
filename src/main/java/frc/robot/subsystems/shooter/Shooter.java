package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.6);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.3);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0195);

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  private void runShooter(double shooterSpeed) {
    io.runShooter(shooterSpeed);
  }

  public Command runShooterTarget() {
    return runEnd(
        () -> {
          io.runShooter(ShooterCalculation.getInstance().getParameters().flywheelSpeed());
        },
        () -> {
          io.runShooter(0);
        });
  }
}
