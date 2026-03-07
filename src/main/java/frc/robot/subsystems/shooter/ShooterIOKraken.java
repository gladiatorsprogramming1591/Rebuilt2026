package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOKraken implements ShooterIO {
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withVelocity(0.0);

  private final StatusSignal<AngularVelocity> shooterRPM;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Temperature> rightLeaderMotorTemp;
  private final StatusSignal<Temperature> rightFollowerMotorTemp;
  private final StatusSignal<Temperature> leftLeaderMotorTemp;
  private final StatusSignal<Temperature> leftFollowerMotorTemp;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;

  private final TalonFX rightShooterLeader =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);

  private final TalonFX leftShooterLeader =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);

  public ShooterIOKraken() {
    shooterRPM = rightShooterLeader.getVelocity();
    shooterAppliedVolts = rightShooterLeader.getMotorVoltage();
    rightLeaderMotorTemp = rightShooterLeader.getDeviceTemp();
    rightFollowerMotorTemp = rightShooterFollower.getDeviceTemp();
    leftLeaderMotorTemp = leftShooterLeader.getDeviceTemp();
    leftFollowerMotorTemp = leftShooterFollower.getDeviceTemp();
    supplyCurrentAmps = rightShooterLeader.getSupplyCurrent();
    torqueCurrentAmps = rightShooterLeader.getTorqueCurrent();

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // tryUntilOk(5, ()-> shooterFollower.getConfigurator().apply(shooterConfig, 0.25));
    var followerConfig = shooterConfig.clone();

    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooterFollower.getConfigurator().apply(shooterConfig, 0.25);
    leftShooterFollower.getConfigurator().apply(followerConfig, 0.25);
    rightShooterLeader.setControl(
        new Follower(rightShooterFollower.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterLeader.setControl(
        new Follower(leftShooterFollower.getDeviceID(), MotorAlignmentValue.Aligned));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;

    rightShooterLeader.getConfigurator().apply(slot0Configs);
    leftShooterLeader.getConfigurator().apply(slot0Configs);
  }

  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    inputs.appliedVoltage = shooterAppliedVolts.getValueAsDouble();
    inputs.rightLeaderMotorTemp = rightLeaderMotorTemp.getValueAsDouble();
    inputs.rightFollowerMotorTemp = rightFollowerMotorTemp.getValueAsDouble();
    inputs.leftLeaderMotorTemp = leftLeaderMotorTemp.getValueAsDouble();
    inputs.leftFollowerMotorTemp = leftFollowerMotorTemp.getValueAsDouble();

    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
  }

  public void runShooterTarget(double shooterVelocity) {
    rightShooterLeader.set(shooterVelocity);
    leftShooterLeader.set(shooterVelocity);
  }

  public boolean shooterAtVelocity(double shooterVelocity) {
    return rightShooterLeader.getVelocity().getValueAsDouble()
            > shooterVelocity - ShooterConstants.SHOOTER_TOLERANCE
        && leftShooterLeader.getVelocity().getValueAsDouble()
            > shooterVelocity - ShooterConstants.SHOOTER_TOLERANCE;
  }

  @Override
  public void setShooterMotorRPM(double rps) {
    rightShooterLeader.setControl(velocityControl.withVelocity(rps));
    leftShooterLeader.setControl(velocityControl.withVelocity(rps));
  }
}
