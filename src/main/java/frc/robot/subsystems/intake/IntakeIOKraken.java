package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKraken implements IntakeIO {
  private final TalonFX intakeLeft = new TalonFX(IntakeConstants.INTAKE_LEFT);
  private final TalonFX intakeRight = new TalonFX(IntakeConstants.INTAKE_RIGHT);
  private final TalonFX deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY);
  private final DigitalInput topLimit = new DigitalInput(IntakeConstants.TOP_DEPLOY_DIO_PORT);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  private final Slot0Configs deploySlot0 = new Slot0Configs();

  private final StatusSignal<Angle> deployAngle = deployMotor.getPosition();
  private final StatusSignal<Current> deploySupplyCurrent = deployMotor.getSupplyCurrent();
  private final StatusSignal<Current> deployTorqueCurrent = deployMotor.getTorqueCurrent();
  private final StatusSignal<AngularVelocity> deployAngularVelocity = deployMotor.getVelocity();
  private final StatusSignal<Temperature> intakeLeftTemp = intakeLeft.getDeviceTemp();
  private final StatusSignal<Temperature> intakeRightTemp = intakeRight.getDeviceTemp();
  private final StatusSignal<AngularVelocity> intakeLeftAngularVelocity = intakeLeft.getVelocity();
  private final StatusSignal<AngularVelocity> intakeRightAngularVelocity =
      intakeRight.getVelocity();

  public IntakeIOKraken() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    PhoenixUtil.tryUntilOk(5, () -> intakeLeft.getConfigurator().apply(intakeConfig, 0.25));
    intakeRight.setControl(new Follower(intakeLeft.getDeviceID(), MotorAlignmentValue.Opposed));

    var deployConfig = new TalonFXConfiguration();
    deployConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_CURRENT_LIMIT;
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    deployMotor.getConfigurator().apply(deployConfig, 0.25);

    deploySlot0.kP = IntakeConstants.kP;
    deploySlot0.kI = IntakeConstants.kI;
    deploySlot0.kD = IntakeConstants.kD;
    deploySlot0.kV = IntakeConstants.kFF;
    deployMotor.getConfigurator().apply(deploySlot0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp,
        intakeLeftAngularVelocity,
        intakeRightAngularVelocity);

    deployMotor.optimizeBusUtilization();
    intakeLeft.optimizeBusUtilization();
    intakeRight.optimizeBusUtilization();
  }

  @Override
  public void setDeploySpeed(double speed) {
    deployMotor.set(speed);
    SmartDashboard.putNumber("Deploy Set Speed", speed);
  }

  @Override
  public void setDeployTorqueCurrentFOC(double current) {
    TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(current);
    deployMotor.setControl(torqueCurrentRequest);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeLeft.set(speed);
    SmartDashboard.putNumber("Intake Speed", speed);
  }

  @Override
  public void stopDeployMotor() {
    deployMotor.stopMotor();
  }

  @Override
  public void stopIntakeMotor() {
    intakeLeft.stopMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        deployAngle,
        deployAngularVelocity,
        deploySupplyCurrent,
        deployTorqueCurrent,
        intakeLeftTemp,
        intakeRightTemp,
        intakeLeftAngularVelocity,
        intakeRightAngularVelocity);

    inputs.deploySpeed = deployAngularVelocity.getValueAsDouble();
    inputs.deployTorqueCurrentFOC = deployTorqueCurrent.getValueAsDouble();
    inputs.deploySupplyCurrent = deploySupplyCurrent.getValueAsDouble();
    inputs.intakeLeftSpeed = intakeLeftAngularVelocity.getValueAsDouble();
    inputs.intakeRightSpeed = intakeRightAngularVelocity.getValueAsDouble();
    inputs.deployPosition = deployAngle.getValueAsDouble();
    inputs.intakeLeftTemp = intakeLeftTemp.getValueAsDouble();
    inputs.intakeRightTemp = intakeRightTemp.getValueAsDouble();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (RobotState.getIntakeMode()) {
      case POSITION:
        // TODO: Needs to be moved out of period into button on SmartDashboard
        if (Constants.tuningMode) {
          deploySlot0.kP = outputs.kP;
          deploySlot0.kI = outputs.kI;
          deploySlot0.kD = outputs.kD;
          deploySlot0.kV = outputs.kFF;
          deployMotor.getConfigurator().apply(deploySlot0);
        }

        deployMotor.setControl(
            positionControl
                .withPosition(outputs.desiredPosition)
                .withSlot(0)
                .withFeedForward(outputs.kFF));
        break;
      case OFF:
        deployMotor.set(0);
      default:
        System.out.println("Intake Apply Outputs Empty Default");
        break;
    }
  }
}
