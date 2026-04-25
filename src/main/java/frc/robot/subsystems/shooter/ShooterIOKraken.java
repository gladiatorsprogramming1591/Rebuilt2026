package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.util.PhoenixUtil;

import static frc.robot.subsystems.shooter.ShooterConstants.coastRPM;
import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_TABLE_KEY;
import static frc.robot.subsystems.shooter.ShooterConstants.UPDATE_CONFIG_NAME;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// spotless:off
public class ShooterIOKraken implements ShooterIO {
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final MotionMagicVelocityVoltage mmVelocityControl = new MotionMagicVelocityVoltage(0).withSlot(0);

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

  private final TalonFX rightShooterLeader =
  new TalonFX(ShooterConstants.RIGHT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX rightShooterFollower =
  new TalonFX(ShooterConstants.RIGHT_SHOOTER_FOLLOWER_MOTOR_ID);
  
  private final TalonFX leftShooterFollower1 =
  new TalonFX(ShooterConstants.LEFT_SHOOTER_LEADER_MOTOR_ID);
  private final TalonFX leftShooterFollower2 =
  new TalonFX(ShooterConstants.LEFT_SHOOTER_FOLLOWER_MOTOR_ID);
  
  private double desiredRPS = 0.0;
  private int tuneConfigsCreated = 0;
  private static final String CURRENT_CONTROL_MODE = "Control Mode";
  private static final double INIT_CONFIG_TIMEOUT = 0.250;
  private static final double TUNED_CONFIG_TIMEOUT = 0.100; // Equivalent to default timeout
  private static final int INIT_CONFIG_MAX_ATTEMPTS = 5;
  private static final int TUNED_CONFIG_MAX_ATTEMPS = 2;

  public ShooterIOKraken() {
    SmartDashboard.putString(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, "N/A");
    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Tune configs created", tuneConfigsCreated);
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune slot0 created", "N/A");
    SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune MM created", "N/A");

    RL_RPS = rightShooterLeader.getVelocity();
    RF_RPS = rightShooterFollower.getVelocity();
    LL_RPS = leftShooterFollower1.getVelocity();
    LF_RPS = leftShooterFollower2.getVelocity();
    RL_appliedVolts = rightShooterLeader.getMotorVoltage();
    RF_appliedVolts = rightShooterFollower.getMotorVoltage();
    LL_appliedVolts = leftShooterFollower1.getMotorVoltage();
    LF_appliedVolts = leftShooterFollower2.getMotorVoltage();
    RL_motorTemp = rightShooterLeader.getDeviceTemp();
    RF_motorTemp = rightShooterFollower.getDeviceTemp();
    LL_motorTemp = leftShooterFollower1.getDeviceTemp();
    LF_motorTemp = leftShooterFollower2.getDeviceTemp();
    RL_supplyCurrent = rightShooterLeader.getSupplyCurrent();
    RF_supplyCurrent = rightShooterFollower.getSupplyCurrent();
    LL_supplyCurrent = leftShooterFollower1.getSupplyCurrent();
    LF_supplyCurrent = leftShooterFollower2.getSupplyCurrent();
    RL_torqueCurrentAmps = rightShooterLeader.getTorqueCurrent();
    RF_torqueCurrentAmps = rightShooterFollower.getTorqueCurrent();
    LL_torqueCurrentAmps = leftShooterFollower1.getTorqueCurrent();
    LF_torqueCurrentAmps = leftShooterFollower2.getTorqueCurrent();

    // rightShooterLeader.optimizeBusUtilization(4, 0.1);
    // rightShooterFollower.optimizeBusUtilization(4, 0.1);

    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_MOTOR_CURRENT_LIMIT;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // i.e. not inverted

    Slot0Configs slot0Configs = rightConfig.Slot0;
    slot0Configs.kP = ShooterConstants.kP.getAsDouble();
    slot0Configs.kI = ShooterConstants.kI.getAsDouble();
    slot0Configs.kD = ShooterConstants.kD.getAsDouble();
    slot0Configs.kS = ShooterConstants.kS.getAsDouble();
    slot0Configs.kV = ShooterConstants.kV.getAsDouble();
    slot0Configs.kA = ShooterConstants.kA.getAsDouble();

    // Motion Magic velocity configs
    // only used when calling MM control request. e.g. new MotionMagicVelocityVoltage(<rps>)
    MotionMagicConfigs motionMagicConfigs = rightConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.kMMAcceleration.getAsDouble();
    motionMagicConfigs.MotionMagicJerk = ShooterConstants.kMMJerk.getAsDouble();

    TalonFXConfiguration leftConfig = rightConfig.clone();
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // i.e. inverted

    PhoenixUtil.tryUntilOk(INIT_CONFIG_MAX_ATTEMPTS, () -> rightShooterLeader.getConfigurator().apply(rightConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(INIT_CONFIG_MAX_ATTEMPTS, () -> rightShooterFollower.getConfigurator().apply(rightConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(INIT_CONFIG_MAX_ATTEMPTS, () -> leftShooterFollower1.getConfigurator().apply(leftConfig, INIT_CONFIG_TIMEOUT));
    PhoenixUtil.tryUntilOk(INIT_CONFIG_MAX_ATTEMPTS, () -> leftShooterFollower2.getConfigurator().apply(leftConfig, INIT_CONFIG_TIMEOUT));

    rightShooterFollower.setControl(
        new Follower(rightShooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    leftShooterFollower1.setControl(
        new StrictFollower(rightShooterLeader.getDeviceID()));
    leftShooterFollower2.setControl(
        new StrictFollower(rightShooterLeader.getDeviceID()));
  }
  // spotless:on

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
    inputs.RPS_RL = RL_RPS.getValueAsDouble();
    inputs.RPS_RF = RF_RPS.getValueAsDouble();
    inputs.RPS_LL = LL_RPS.getValueAsDouble();
    inputs.RPS_LF = LF_RPS.getValueAsDouble();
    inputs.RPM_RL = RL_RPS.getValueAsDouble() * 60;
    inputs.RPM_RF = RF_RPS.getValueAsDouble() * 60;
    inputs.RPM_LL = LL_RPS.getValueAsDouble() * 60;
    inputs.RPM_LF = LF_RPS.getValueAsDouble() * 60;
    inputs.appliedVolts_RL = RL_appliedVolts.getValueAsDouble();
    inputs.appliedVolts_RF = RF_appliedVolts.getValueAsDouble();
    inputs.appliedVolts_LL = LL_appliedVolts.getValueAsDouble();
    inputs.appliedVolts_LF = LF_appliedVolts.getValueAsDouble();
    inputs.motorTemp_RL = RL_motorTemp.getValueAsDouble();
    inputs.motorTemp_RF = RF_motorTemp.getValueAsDouble();
    inputs.motorTemp_LL = LL_motorTemp.getValueAsDouble();
    inputs.motorTemp_LF = LF_motorTemp.getValueAsDouble();
    inputs.supplyCurrent_RL = RL_supplyCurrent.getValueAsDouble();
    inputs.supplyCurrent_RF = RF_supplyCurrent.getValueAsDouble();
    inputs.supplyCurrent_LL = LL_supplyCurrent.getValueAsDouble();
    inputs.supplyCurrent_LF = LF_supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps_RL = RL_torqueCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps_RF = RF_torqueCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps_LL = LL_torqueCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps_LF = LF_torqueCurrentAmps.getValueAsDouble();
    inputs.shooterAtVelocity = rightShooterAtVelocity().getAsBoolean();
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    desiredRPS = outputs.desiredVelocityRPM / 60;

    SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Desired RPS", desiredRPS);

    switch (RobotState.getShooterMode()) {
      case ON: case IDLE:
        if (outputs.useMotionMagic)
        {
          rightShooterLeader.setControl(mmVelocityControl.withVelocity(desiredRPS));
          SmartDashboard.putString(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, MotionMagicVelocityVoltage.class.getName());
        } else
        {
          rightShooterLeader.setControl(velocityControl.withVelocity(desiredRPS));
          SmartDashboard.putString(SHOOTER_TABLE_KEY + CURRENT_CONTROL_MODE, VelocityVoltage.class.getName());
        }
        break;

      case DUTYCYCLE:
        rightShooterLeader.set(outputs.desiredDutyCycle);
        break;

      case OFF:
        rightShooterLeader.stopMotor();
        break;

      default:
        System.out.println("Shooter Apply Outputs Empty Default");
        break;
    }
  }

  // @Override
  // public void runShooterDutyCycle(double dutyCycle) {
  //   rightShooterLeader.set(dutyCycle);
  // }

  @Override
  public BooleanSupplier rightShooterAtVelocity() {
    return rightShooterAtVelocity(() -> desiredRPS);
  }

  @Override
  public BooleanSupplier rightShooterAtVelocity(DoubleSupplier targetRPS) {
    return (() ->
        (Math.abs(RL_RPS.getValueAsDouble() - targetRPS.getAsDouble()) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS));
  }

  @Override
  public BooleanSupplier rightShooterBelowCoastRPM() {
    double coastRPS = coastRPM.getAsDouble() / 60;
    return (() ->
        RL_RPS.getValueAsDouble() < (coastRPS + ShooterConstants.FLYWHEEL_TOLERANCE_RPS));
  }

  @Override
  public void tuneMotorConfigs(ShooterIOOutputs outputs)
  {
    if (SmartDashboard.getBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false))
      {
      SmartDashboard.putBoolean(SHOOTER_TABLE_KEY + UPDATE_CONFIG_NAME, false);
      
      TalonFXConfiguration tunedConfigs = this.createTunedMotorConfig(outputs);
      Slot0Configs slot0 = tunedConfigs.Slot0;
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> rightShooterLeader.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> leftShooterFollower1.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> rightShooterFollower.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> leftShooterFollower2.getConfigurator().apply(slot0, TUNED_CONFIG_TIMEOUT)); 
      
      MotionMagicConfigs mmConfigs = tunedConfigs.MotionMagic;
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> rightShooterLeader.getConfigurator().apply(mmConfigs, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> leftShooterFollower1.getConfigurator().apply(mmConfigs, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> rightShooterFollower.getConfigurator().apply(mmConfigs, TUNED_CONFIG_TIMEOUT));
      PhoenixUtil.tryUntilOk(TUNED_CONFIG_MAX_ATTEMPS, () -> leftShooterFollower2.getConfigurator().apply(mmConfigs, TUNED_CONFIG_TIMEOUT));      
    }
  }

  /**
   * Creates a TalonFX configuration with the latest tunable settings for <b>all shooter motors</b>.
   * 
   * <ul>
   *  <li> <b>Updated Configurations:</b>
   *    <ul>
          <li> {@code Slot0Configs}: P, I, D, S, V, and A
          <li> {@code MotionMagicConfigs}: Acceleration and Jerk
        </ul>
   * </ul>
   * 
   * @param outputs Shooter outputs where the tunable configurations are accessable
   * @return Tuned TalonFX configuration
   * @see frc.robot.subsystems.shooter.Shooter#periodic Shooter periodic() where tunable configs are saved into outputs
   * @see frc.robot.util.LoggedTunableNumber LoggedTunableNumber
   */
  private TalonFXConfiguration createTunedMotorConfig(ShooterIOOutputs outputs)
  {
      TalonFXConfiguration configs = new TalonFXConfiguration();
      Slot0Configs slot0Configs = configs.Slot0;
      slot0Configs.kP = outputs.kP;
      slot0Configs.kI = outputs.kI;
      slot0Configs.kD = outputs.kD;
      slot0Configs.kS = outputs.kS;
      slot0Configs.kV = outputs.kV;
      slot0Configs.kA = outputs.kA;
      MotionMagicConfigs mmConfigs = configs.MotionMagic;
      mmConfigs.MotionMagicAcceleration = outputs.kMMAcceleration;
      mmConfigs.MotionMagicJerk = outputs.kMMJerk;

      SmartDashboard.putNumber(SHOOTER_TABLE_KEY + "Tune configs created", ++tuneConfigsCreated);
      SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune slot0 created", configs.Slot0.toString());
      SmartDashboard.putString(SHOOTER_TABLE_KEY + "Tune MM created", configs.MotionMagic.toString());
      return configs;
  }

  // =======================UNTESTED/UNUSED METHODS=======================

  /**
   * =======================UNTESTED=======================
   * ========================UNUSED========================
   *
   * @return
   */
  @Override
  public BooleanSupplier leftShooterAtVelocity() {

    return (() ->
        leftShooterFollower1.getVelocity().getValueAsDouble()
            > desiredRPS - ShooterConstants.FLYWHEEL_TOLERANCE_RPS);
  }

  /**
   * =======================UNTESTED=======================
   * ========================UNUSED========================
   *
   * @return
   */
  @Override
  public BooleanSupplier bothShootersAtVelocity() {

    return () -> leftShooterAtVelocity().getAsBoolean() && rightShooterAtVelocity().getAsBoolean();
  }
}
