package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

@SuppressWarnings("unused")
public class LoggedTunableBoolean implements BooleanSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final boolean tuningEnabled;
  private final String key;

  private boolean hasDefault = false;
  private boolean defaultValue;
  private LoggedNetworkBoolean dashboardBoolean;
  private Map<Integer, Boolean> lastHasChangedValues;

  public LoggedTunableBoolean(String dashboardKey) {
    this.tuningEnabled = Constants.tuningMode && !Constants.disableHAL;
    this.key = tuningEnabled ? TABLE_KEY + "/" + dashboardKey : null;
  }

  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey, defaultValue, Constants.tuningMode && !Constants.disableHAL);
  }

  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue, boolean tuningEnabled) {
    this.tuningEnabled = tuningEnabled;
    this.key = tuningEnabled ? TABLE_KEY + "/" + dashboardKey : null;
    initDefault(defaultValue);
  }

  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      if (tuningEnabled) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  public boolean get() {
    if (!hasDefault) {
      return false;
    }

    return tuningEnabled ? dashboardBoolean.get() : defaultValue;
  }

  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Map<Integer, Boolean> lastValues = getLastHasChangedValues();

    Boolean lastValue = lastValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  private Map<Integer, Boolean> getLastHasChangedValues() {
    if (lastHasChangedValues == null) {
      lastHasChangedValues = new HashMap<>();
    }

    return lastHasChangedValues;
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}