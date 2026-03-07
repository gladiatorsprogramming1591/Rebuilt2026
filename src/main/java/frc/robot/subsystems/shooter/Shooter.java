package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIO.ShooterIOOutputs outputs = new ShooterIO.ShooterIOOutputs();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.6);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.3);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0195);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    switch (RobotState.getShooterMode()) {
      case ON -> io.setShooterMotorRPM(50);
      case IDLE -> io.setShooterMotorRPM(25); // NOT REAL, JUST HALF VOLTAGE
      case OFF -> io.setShooterMotorRPM(0);
      default -> {
        System.out.println("Illegal Shooter mode : " + RobotState.getShooterMode());
        io.setShooterMotorRPM(0);
      }
    }
  }

  private void runShooter(double shooterSpeed) {
    io.runShooterTarget(shooterSpeed);
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

  public Command runShooterVelocity(double velocity) {
    SmartDashboard.putNumber("Shooter Vel", velocity);
    return runEnd(
        () -> {
          io.runShooterTarget(velocity);
        },
        () -> {
          io.runShooterTarget(0);
        });
  }

  // public boolean isShooterAtVelocity(double velocity) {
  //   return MathUtil.isNear(
  //     velocity,
  //     , velocity)
  // }

}
