package frc.robot.util;
// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@SuppressWarnings("unused")
public class LoggedTunableBoolean implements BooleanSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue;
  private LoggedNetworkBoolean dashboardBoolean;
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableBoolean
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableBoolean with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode && !Constants.disableHAL) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public boolean get() {
    if (!hasDefault) {
      return false;
    } else {
      return Constants.tuningMode && !Constants.disableHAL ? dashboardBoolean.get() : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableBooleans All tunable numbers to check
   */
  // public static void ifChanged(
  //     int id, Consumer<boolean[]> action, LoggedTunableBoolean... tunableBooleans) {
  //   if (Arrays.stream(tunableBooleans).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id)))
  // {
  //     // TODO: Map to boolean array (if possible) then uncomment both this method and the one
  // below
  //
  // action.accept(Arrays.stream(tunableBooleans).mapToDouble(LoggedTunableBoolean::get).toArray());
  //   }
  // }

  /** Runs action if any of the tunableNumbers have changed */
  // public static void ifChanged(int id, Runnable action, LoggedTunableBoolean... tunableBooleans)
  // {
  //   ifChanged(id, values -> action.run(), tunableBooleans);
  // }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
