package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhoenixUtil;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterIOKraken implements ShooterIO {
  // private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage magicVelocityControl =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<AngularVelocity> RL_RPS;
  private final StatusSignal<AngularVelocity> RF_RPS;
  private final StatusSignal<AngularVelocity> LL_RPS;
  private final StatusSignal<AngularVelocity> LF_RPS;
  private final StatusSignal<Voltage> RL_appliedVolts;
  private final StatusSignal<Voltage> RF_appliedVolts;
  private final StatusSignal<Voltage> LL_appliedVolts;
  private final StatusSignal<Voltage> LF_appliedVolts;
  private final StatusSignal<Temperature> RL_motorTemp;
  private final StatusSignal<Temperature> RF_motorTemp;
  private final StatusSignal<Temperature> LL_motorTemp;
  private final StatusSignal<Temperature> LF_motorTemp;
  private final StatusSignal<Current> RL_supplyCurrent;
  private final StatusSignal<Current> RF_supplyCurrent;
  private final StatusSignal<Current> LL_supplyCurrent;
  private final StatusSignal<Current> LF_supplyCurrent;
  private final StatusSignal<Current> RL_torqueCurrentAmps;
  private final StatusSignal<Current> RF_torqueCurrentAmps;
  private final StatusSignal<Current> LL_torqueCurrentAmps;
  private final StatusSignal<Current> LF_torqueCurrentAmps;
  private double lastCommandedVelocity = 0.0;

  private final TalonFX rightShooterLeader =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
      new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);

  private final TalonFX leftShooterLeader =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower =
      new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);

  public ShooterIOKraken() {
    RL_RPS = rightShooterLeader.getVelocity();
    RF_RPS = rightShooterFollower.getVelocity();
    LL_RPS = leftShooterLeader.getVelocity();
    LF_RPS = leftShooterFollower.getVelocity();
    RL_appliedVolts = rightShooterLeader.getMotorVoltage();
    RF_appliedVolts = rightShooterFollower.getMotorVoltage();
    LL_appliedVolts = leftShooterLeader.getMotorVoltage();
    LF_appliedVolts = leftShooterFollower.getMotorVoltage();
    RL_motorTemp = rightShooterLeader.getDeviceTemp();
    RF_motorTemp = rightShooterFollower.getDeviceTemp();
    LL_motorTemp = leftShooterLeader.getDeviceTemp();
    LF_motorTemp = leftShooterFollower.getDeviceTemp();
    RL_supplyCurrent = rightShooterLeader.getSupplyCurrent();
    RF_supplyCurrent = rightShooterFollower.getSupplyCurrent();
    LL_supplyCurrent = leftShooterLeader.getSupplyCurrent();
    LF_supplyCurrent = leftShooterFollower.getSupplyCurrent();
    RL_torqueCurrentAmps = rightShooterLeader.getTorqueCurrent();
    RF_torqueCurrentAmps = rightShooterFollower.getTorqueCurrent();
    LL_torqueCurrentAmps = leftShooterLeader.getTorqueCurrent();
    LF_torqueCurrentAmps = leftShooterFollower.getTorqueCurrent();

    // rightShooterLeader.optimizeBusUtilization(4, 0.1);
    // rightShooterFollower.optimizeBusUtilization(4, 0.1);

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
    PhoenixUtil.tryUntilOk(5, () -> rightShooterLeader.getConfigurator().apply(rightConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> leftShooterLeader.getConfigurator().apply(leftConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5, () -> rightShooterFollower.getConfigurator().apply(rightConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> leftShooterFollower.getConfigurator().apply(leftConfig, 0.25));
    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterFollower.setControl(
        new Follower(leftShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        RL_RPS,
        RF_RPS,
        LL_RPS,
        LF_RPS,
        RL_appliedVolts,
        RF_appliedVolts,
        LL_appliedVolts,
        LF_appliedVolts,
        RL_motorTemp,
        RF_motorTemp,
        LL_motorTemp,
        LF_motorTemp,
        RL_supplyCurrent,
        RF_supplyCurrent,
        LL_supplyCurrent,
        LF_supplyCurrent,
        RL_torqueCurrentAmps,
        RF_torqueCurrentAmps,
        LL_torqueCurrentAmps,
        LF_torqueCurrentAmps);
    inputs.RL_RPS = RL_RPS.getValueAsDouble();
    inputs.RF_RPS = RF_RPS.getValueAsDouble();
    inputs.LL_RPS = LL_RPS.getValueAsDouble();
    inputs.LF_RPS = LF_RPS.getValueAsDouble();
    inputs.RL_appliedVolts = RL_appliedVolts.getValueAsDouble();
    inputs.RF_appliedVolts = RF_appliedVolts.getValueAsDouble();
    inputs.LL_appliedVolts = LL_appliedVolts.getValueAsDouble();
    inputs.LF_appliedVolts = LF_appliedVolts.getValueAsDouble();
    inputs.RL_motorTemp = RL_motorTemp.getValueAsDouble();
    inputs.RF_motorTemp = RF_motorTemp.getValueAsDouble();
    inputs.LL_motorTemp = LL_motorTemp.getValueAsDouble();
    inputs.LF_motorTemp = LF_motorTemp.getValueAsDouble();
    inputs.RL_supplyCurrent = RL_supplyCurrent.getValueAsDouble();
    inputs.RF_supplyCurrent = RF_supplyCurrent.getValueAsDouble();
    inputs.LL_supplyCurrent = LL_supplyCurrent.getValueAsDouble();
    inputs.LF_supplyCurrent = LF_supplyCurrent.getValueAsDouble();
    inputs.RL_torqueCurrentAmps = RL_torqueCurrentAmps.getValueAsDouble();
    inputs.RF_torqueCurrentAmps = RF_torqueCurrentAmps.getValueAsDouble();
    inputs.LL_torqueCurrentAmps = LL_torqueCurrentAmps.getValueAsDouble();
    inputs.LF_torqueCurrentAmps = LF_torqueCurrentAmps.getValueAsDouble();
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
    lastCommandedVelocity = outputs.desiredVelocityRPM / 60;

    Logger.recordOutput("Shooter/lastCommandedVelocity", lastCommandedVelocity);
    SmartDashboard.putNumber("lastCommandedVelocity", lastCommandedVelocity);
    rightShooterLeader.setControl(
        magicVelocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
    leftShooterLeader.setControl(
        magicVelocityControl.withVelocity(outputs.desiredVelocityRPM / 60));
  }

  @Override
  public void runShooterDutyCycle(double dutyCycle) {
    rightShooterLeader.set(dutyCycle);
    leftShooterLeader.set(dutyCycle);
  }

  @Override
  public BooleanSupplier shooterAtVelocity() {

    return (() ->
        rightShooterLeader.getVelocity().getValueAsDouble()
                > lastCommandedVelocity - ShooterConstants.SHOOTER_TOLERANCE
            && leftShooterLeader.getVelocity().getValueAsDouble()
                > lastCommandedVelocity - ShooterConstants.SHOOTER_TOLERANCE);
  }

  @Override
  public void setShooterMotorRPM(double rps) {
    lastCommandedVelocity = rps;
    rightShooterLeader.setControl(magicVelocityControl.withVelocity(rps));
    leftShooterLeader.setControl(magicVelocityControl.withVelocity(rps));
  }
}
