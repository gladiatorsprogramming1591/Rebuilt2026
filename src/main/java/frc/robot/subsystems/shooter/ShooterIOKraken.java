package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage magicVelocityControl =
      new MotionMagicVelocityVoltage(0).withSlot(0);

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

    var slot0Configs = rightConfig.Slot0;

    slot0Configs.kS = 0.0; // Add 0.0 V output to overcome static friction
    slot0Configs.kV = 0.125; // A velocity target of 1 rps results in 0.125 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.65; // An error of 1 rps results in 0.65 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = rightConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 200 rps/s (0.50 (?) seconds to max)
    motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 2000 rps/s/s (0.2 (?) seconds)

    // tryUntilOk(5, ()-> shooterFollower.getConfigurator().apply(shooterConfig, 0.25));
    var leftConfig = rightConfig.clone();

    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooterLeader.getConfigurator().apply(rightConfig, 0.25);
    leftShooterLeader.getConfigurator().apply(leftConfig, 0.25);
    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterFollower.setControl(
        new Follower(leftShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
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

    // rightShooterLeader.getConfigurator().apply(slot0Configs); // Cannot call periodicly
    // leftShooterLeader.getConfigurator().apply(slot0Configs);

    // rightShooterLeader.setControl(velocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
    // leftShooterLeader.setControl(velocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
    rightShooterLeader.setControl(
        magicVelocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
    leftShooterLeader.setControl(
        magicVelocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
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

  @Override
  public void setShooterMotorRPM(double rps) {
    rightShooterLeader.setControl(velocityControl.withVelocity(rps));
    leftShooterLeader.setControl(velocityControl.withVelocity(rps));
  }
}
