// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.robotInitConstants;
import frc.robot.subsystems.shooter.ShooterCalculation;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double lowBatteryVoltage = 11.0;
  private static final double lowBatteryDisabledTime = 2.0;

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double autoStart;
  private boolean autoMessagePrinted;

  private final Timer disabledTimer = new Timer();
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
          AlertType.kWarning);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Clear launching parameters so that they are refreshed next getParameters()
    var shooterCalculation = ShooterCalculation.getInstance();
    shooterCalculation.clearLaunchingParameters();

    // Log launching parameters
    Logger.recordOutput("ShooterCalculation/Parameters", shooterCalculation.getParameters());
    Logger.recordOutput(
        "ShooterCalculation/HoodAngleOffsetDeg", shooterCalculation.getHoodAngleOffsetDeg());
    String formattedOffset = String.format("%.1f", shooterCalculation.getHoodAngleOffsetDeg());
    if (formattedOffset.equals("-0.0")) {
      formattedOffset = "0.0";
    }
    SmartDashboard.putString("Launch Hood Angle Offset", formattedOffset);

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() > 0.0
        && RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();

    // Official shift info
    SmartDashboard.putString(
        "Shift/Official/CurrentShift",
        HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putNumber(
        "Shift/Official/Remaining", HubShiftUtil.getOfficialShiftInfo().remainingTime());
    SmartDashboard.putNumber(
        "Shift/Official/RemainingTime", HubShiftUtil.getShiftedShiftInfo().remainingTime());

    SmartDashboard.putNumber(
        "Shift/Official/ElapsedTime", Math.round(official.elapsedTime() * 10) / 10.0);
    SmartDashboard.putBoolean("Shift/Official/Active", official.active());

    // Update RobotContainer dashboard outputs
    robotContainer.updateDashboardOutputs();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Print auto duration
    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    if (robotInitConstants.isCompBot) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-two")
          .getEntry("throttle_set")
          .setNumber(100);
      NetworkTableInstance.getDefault()
          .getTable("limelight-three")
          .getEntry("throttle_set")
          .setNumber(100);
    } else {
      NetworkTableInstance.getDefault()
          .getTable("limelight-one")
          .getEntry("throttle_set")
          .setNumber(100);
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (robotInitConstants.isCompBot) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-two")
          .getEntry("throttle_set")
          .setNumber(0);
      NetworkTableInstance.getDefault()
          .getTable("limelight-three")
          .getEntry("throttle_set")
          .setNumber(0);
    } else {
      NetworkTableInstance.getDefault()
          .getTable("limelight-one")
          .getEntry("throttle_set")
          .setNumber(0);
    }

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    if (robotInitConstants.isCompBot) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-two")
          .getEntry("throttle_set")
          .setNumber(0);
      NetworkTableInstance.getDefault()
          .getTable("limelight-three")
          .getEntry("throttle_set")
          .setNumber(0);
    } else {
      NetworkTableInstance.getDefault()
          .getTable("limelight-one")
          .getEntry("throttle_set")
          .setNumber(0);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    if (robotInitConstants.isCompBot) {
      NetworkTableInstance.getDefault()
          .getTable("limelight-two")
          .getEntry("throttle_set")
          .setNumber(0);
      NetworkTableInstance.getDefault()
          .getTable("limelight-three")
          .getEntry("throttle_set")
          .setNumber(0);
    } else {
      NetworkTableInstance.getDefault()
          .getTable("limelight-one")
          .getEntry("throttle_set")
          .setNumber(0);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
