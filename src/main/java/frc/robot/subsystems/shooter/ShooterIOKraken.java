package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
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

  private final StatusSignal<AngularVelocity> shooterRPS;
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
    shooterRPS = rightShooterLeader.getVelocity();
    shooterAppliedVolts = rightShooterLeader.getMotorVoltage();
    rightLeaderMotorTemp = rightShooterLeader.getDeviceTemp();
    rightFollowerMotorTemp = rightShooterFollower.getDeviceTemp();
    leftLeaderMotorTemp = leftShooterLeader.getDeviceTemp();
    leftFollowerMotorTemp = leftShooterFollower.getDeviceTemp();
    supplyCurrentAmps = rightShooterLeader.getSupplyCurrent();
    torqueCurrentAmps = rightShooterLeader.getTorqueCurrent();

    var rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // tryUntilOk(5, ()-> shooterFollower.getConfigurator().apply(shooterConfig, 0.25));
    var leftConfig = rightConfig.clone();

    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooterLeader.getConfigurator().apply(rightConfig, 0.25);
    leftShooterLeader.getConfigurator().apply(leftConfig, 0.25);
    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterFollower.setControl(
        new Follower(leftShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.kP;
    slot0Configs.kI = ShooterConstants.kI;
    slot0Configs.kD = ShooterConstants.kD;
    slot0Configs.kV = ShooterConstants.kV;

    rightShooterLeader.getConfigurator().apply(slot0Configs);
    leftShooterLeader.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterRPS,
        shooterAppliedVolts,
        rightLeaderMotorTemp,
        rightFollowerMotorTemp,
        leftLeaderMotorTemp,
        leftFollowerMotorTemp,
        supplyCurrentAmps,
        torqueCurrentAmps);
    inputs.appliedVoltage = shooterAppliedVolts.getValueAsDouble();
    inputs.rightLeaderMotorTemp = rightLeaderMotorTemp.getValueAsDouble();
    inputs.rightFollowerMotorTemp = rightFollowerMotorTemp.getValueAsDouble();
    inputs.leftLeaderMotorTemp = leftLeaderMotorTemp.getValueAsDouble();
    inputs.leftFollowerMotorTemp = leftFollowerMotorTemp.getValueAsDouble();
    inputs.velocityRPM = shooterRPS.getValueAsDouble() * 60;

    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = outputs.kP;
    slot0Configs.kI = outputs.kI;
    slot0Configs.kD = outputs.kD;
    slot0Configs.kV = outputs.kV;

    rightShooterLeader.getConfigurator().apply(slot0Configs);
    leftShooterLeader.getConfigurator().apply(slot0Configs);

    rightShooterLeader.setControl(velocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
    leftShooterLeader.setControl(velocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
  }

  @Override
  public void runShooterVelocity(double shooterVelocity) {
    rightShooterLeader.set(shooterVelocity);
    leftShooterLeader.set(shooterVelocity);
  }

  @Override
  public boolean shooterAtVelocity(double shooterVelocity) {
    return rightShooterLeader.getVelocity().getValueAsDouble()
            > shooterVelocity - ShooterConstants.SHOOTER_TOLERANCE
        && leftShooterLeader.getVelocity().getValueAsDouble()
            > shooterVelocity - ShooterConstants.SHOOTER_TOLERANCE;
  }

  // @Override
  // public void setShooterMotorRPM(double rpm) {
  //   rightShooterLeader.setControl(velocityControl.withVelocity(rpm));
  //   leftShooterLeader.setControl(velocityControl.withVelocity(rpm));
  // }
}
