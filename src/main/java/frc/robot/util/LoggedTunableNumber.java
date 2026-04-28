package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@SuppressWarnings("unused")
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final boolean tuningEnabled;
  private final String key;

  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues;

  public LoggedTunableNumber(String dashboardKey) {
    this(dashboardKey, Constants.tuningMode && !Constants.disableHAL);
  }

  public LoggedTunableNumber(String dashboardKey, boolean tuningEnabled) {
    this.tuningEnabled = tuningEnabled;
    this.key = tuningEnabled ? TABLE_KEY + "/" + dashboardKey : null;
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey, defaultValue, Constants.tuningMode && !Constants.disableHAL);
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue, boolean tuningEnabled) {
    this(dashboardKey, tuningEnabled);
    initDefault(defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      if (tuningEnabled) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    }

    return tuningEnabled ? dashboardNumber.get() : defaultValue;
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    Map<Integer, Double> lastValues = getLastHasChangedValues();

    Double lastValue = lastValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  private Map<Integer, Double> getLastHasChangedValues() {
    if (lastHasChangedValues == null) {
      lastHasChangedValues = new HashMap<>();
    }

    return lastHasChangedValues;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      Stream<LoggedTunableNumber> stream = Arrays.stream(tunableNumbers);
      DoubleStream doubleStream = stream.mapToDouble(LoggedTunableNumber::get);
      double[] doubleArray = doubleStream.toArray();
      action.accept(doubleArray);
    }
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}