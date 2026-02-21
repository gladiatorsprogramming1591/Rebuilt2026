package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX rightShooterLeader =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);

  private final TalonFX leftShooterLeader =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);

  public ShooterIOKraken() {
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
  }

  @Override
  public void setShooterSpeedVelocity(double shooterVelocity) {
    rightShooterLeader.set(shooterVelocity);
    leftShooterLeader.set(shooterVelocity);
  }
}
