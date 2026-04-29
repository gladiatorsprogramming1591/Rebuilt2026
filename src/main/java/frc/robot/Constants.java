// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true; // TODO: Remove before Tech Valley
  public static final boolean disableHAL = false;

  public static final class Tuning {
    private static final boolean ENABLED = tuningMode && !disableHAL;

    public static final boolean DRIVE = ENABLED && true;
    public static final boolean DRIVE_COMMANDS = ENABLED && true;

    public static final boolean HOOD = ENABLED && false;
    public static final boolean SHOOTER = ENABLED && false;

    // This is part of shooter tuning, but separate because it creates a ton of map tunables.
    public static final boolean SHOOTER_CALCULATION = SHOOTER && false;
    public static final boolean SOTM = true;

    public static final boolean INTAKE = ENABLED && true;
    public static final boolean HOPPER = ENABLED && true;
    public static final boolean KICKER = ENABLED && false;
    public static final boolean AUTO = ENABLED && false;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static double loopPeriodSecs = 0.02;

  public class robotInitConstants {
    public static final DigitalInput dIO_port = new DigitalInput(9);
    public static final boolean isCompBot = dIO_port.get();
  }
}
