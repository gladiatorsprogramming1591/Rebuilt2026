package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Driver controller wrapper that exposes Xbox-style button names.
 *
 * <p>When PS4 mode is selected, the PS4 buttons are mapped to the equivalent Xbox-style names so
 * RobotContainer does not need to care which physical controller is plugged in.
 */
public class DriverController {
  public enum ControllerType {
    XBOX,
    PS4
  }

  private final CommandXboxController xboxController;
  private final CommandPS4Controller ps4Controller;
  private final Supplier<ControllerType> controllerTypeSupplier;

  public DriverController(int port, Supplier<ControllerType> controllerTypeSupplier) {
    xboxController = new CommandXboxController(port);
    ps4Controller = new CommandPS4Controller(port);
    this.controllerTypeSupplier = controllerTypeSupplier;
  }

  private ControllerType getControllerType() {
    ControllerType selectedType = controllerTypeSupplier.get();
    return selectedType != null ? selectedType : ControllerType.XBOX;
  }

  private boolean isPs4Controller() {
    return getControllerType() == ControllerType.PS4;
  }

  private Trigger mapControl(BooleanSupplier xboxControl, BooleanSupplier ps4Control) {
    return new Trigger(() -> isPs4Controller() ? ps4Control.getAsBoolean() : xboxControl.getAsBoolean());
  }

  public Trigger a() {
    return mapControl(xboxController.a(), ps4Controller.cross());
  }

  public Trigger b() {
    return mapControl(xboxController.b(), ps4Controller.circle());
  }

  public Trigger x() {
    return mapControl(xboxController.x(), ps4Controller.square());
  }

  public Trigger y() {
    return mapControl(xboxController.y(), ps4Controller.triangle());
  }

  public Trigger leftBumper() {
    return mapControl(xboxController.leftBumper(), ps4Controller.L1());
  }

  public Trigger rightBumper() {
    return mapControl(xboxController.rightBumper(), ps4Controller.R1());
  }

  public Trigger leftTrigger() {
    return mapControl(xboxController.leftTrigger(), ps4Controller.L2());
  }

  public Trigger rightTrigger() {
    return mapControl(xboxController.rightTrigger(), ps4Controller.R2());
  }

  public Trigger leftStick() {
    return mapControl(xboxController.leftStick(), ps4Controller.L3());
  }

  public Trigger rightStick() {
    return mapControl(xboxController.rightStick(), ps4Controller.R3());
  }

  public Trigger start() {
    return mapControl(xboxController.start(), ps4Controller.options());
  }

  public Trigger back() {
    return mapControl(xboxController.back(), ps4Controller.share());
  }

  public Trigger povUp() {
    return mapControl(xboxController.povUp(), ps4Controller.povUp());
  }

  public Trigger povDown() {
    return mapControl(xboxController.povDown(), ps4Controller.povDown());
  }

  public Trigger povLeft() {
    return mapControl(xboxController.povLeft(), ps4Controller.povLeft());
  }

  public Trigger povRight() {
    return mapControl(xboxController.povRight(), ps4Controller.povRight());
  }

  public double getLeftX() {
    return isPs4Controller() ? ps4Controller.getLeftX() : xboxController.getLeftX();
  }

  public double getLeftY() {
    return isPs4Controller() ? ps4Controller.getLeftY() : xboxController.getLeftY();
  }

  public double getRightX() {
    return isPs4Controller() ? ps4Controller.getRightX() : xboxController.getRightX();
  }

  public double getRightY() {
    return isPs4Controller() ? ps4Controller.getRightY() : xboxController.getRightY();
  }

  public void setRumble(RumbleType rumbleType, double value) {
    xboxController.getHID().setRumble(rumbleType, value);
    ps4Controller.getHID().setRumble(rumbleType, value);
  }
}