package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Bounds;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

/**
 * Calculates shooter hood, flywheel, and robot aiming targets.
 *
 * <p>The calculation starts from the current robot pose, compensates for robot motion during the
 * ball's time of flight, then reads hood and flywheel targets from distance-based lookup maps.
 */
@ExtensionMethod({GeomUtil.class})
public class ShooterCalculation {
  private static final String TABLE_KEY = "ShooterCalculation/";
  private static final int LOOKAHEAD_ITERATIONS = 20;

  private static ShooterCalculation instance;

  @Getter private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  /**
   * Scales velocity compensation used for robot aiming.
   *
   * <p>Tune this when left/right SOTM shots are off but in/out distance compensation looks correct.
   */
  private static final LoggedTunableNumber aimVelocityCompensationScalar =
      new LoggedTunableNumber(TABLE_KEY + "AimVelocityCompensationScalar", 0.2, Constants.Tuning.SOTM);

  /**
   * Scales velocity compensation used for hood and flywheel distance lookup.
   *
   * <p>Tune this when driving toward/away from the goal causes shots to miss long or short.
   */
  private static final LoggedTunableNumber distanceVelocityCompensationScalar =
      new LoggedTunableNumber(TABLE_KEY + "DistanceVelocityCompensationScalar", 0, Constants.Tuning.SOTM);

  /**
   * Scales acceleration compensation used for robot aiming.
   *
   * <p>This defaults to zero so acceleration can be logged first and enabled only after testing.
   */
  private static final LoggedTunableNumber aimAccelerationCompensationScalar =
      shooterCalcTunable(TABLE_KEY + "AimAccelerationCompensationScalar", 0.0);

  /**
   * Scales acceleration compensation used for hood and flywheel distance lookup.
   *
   * <p>This defaults to zero so acceleration can be logged first and enabled only after testing.
   */
  private static final LoggedTunableNumber distanceAccelerationCompensationScalar =
      shooterCalcTunable(TABLE_KEY + "DistanceAccelerationCompensationScalar", 0.0);


  /** Optional simple SOTM model that shifts the aim target by field velocity * time of flight. */
  private static final LoggedTunableBoolean simpleTargetShiftEnabled =
      new LoggedTunableBoolean(
          TABLE_KEY + "SimpleTargetShift/Enabled",
          false,
          Constants.Tuning.SOTM);

  /** Scalar applied to the simple velocity target shift. Start small if testing. */
  private static final LoggedTunableNumber simpleTargetShiftScalar =
      new LoggedTunableNumber(
          TABLE_KEY + "SimpleTargetShift/Scalar",
          1.0,
          Constants.Tuning.SOTM);

  /** Weight for acceleration derived from change in measured field-relative velocity. */
  private static final LoggedTunableNumber velocityDerivedAccelerationWeight =
      shooterCalcTunable(TABLE_KEY + "Acceleration/VelocityDerivedWeight", 1.0);

  /** Weight for field-relative acceleration supplied from the IMU path in RobotState. */
  private static final LoggedTunableNumber imuAccelerationWeight =
      shooterCalcTunable(TABLE_KEY + "Acceleration/ImuWeight", 0.0);

  private double lastHoodAngle = Double.NaN;
  private Rotation2d lastDriveAngle;

  private static final String updateHoodMapKey =
      TABLE_KEY + "UpdateMaps/HoodAngleMap/Update Hood Angles";
  private static final String updateFlywheelMapKey =
      TABLE_KEY + "UpdateMaps/FlyWheelSpeedMap/Update Flywheel Speeds";
  private static final String updatePassingHoodMapKey =
      TABLE_KEY + "UpdateMaps/PassingHoodMap/Update Passing Hood Angles";
  private static final String updatePassingFlywheelMapKey =
      TABLE_KEY + "UpdateMaps/PassingFlywheelMap/Update Passing Flywheel Speeds";

  /**
   * Returns the singleton shooter calculation instance.
   *
   * @return shooter calculation instance
   */
  public static ShooterCalculation getInstance() {
    if (instance == null) {
      instance = new ShooterCalculation();
    }

    return instance;
  }

  private static LoggedTunableNumber shooterCalcTunable(String key, double defaultValue) {
    return new LoggedTunableNumber(key, defaultValue, Constants.Tuning.SHOOTER_CALCULATION);
  }

  /** Calculated shooter and aiming parameters for the current loop. */
  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  /** Preset hood and flywheel values exposed for tuning/testing. */
  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  private record MotionOffset(
      double velocityX, double velocityY, double accelerationX, double accelerationY) {
    double totalX() {
      return velocityX + accelerationX;
    }

    double totalY() {
      return velocityY + accelerationY;
    }
  }

  // Cache parameters so multiple subsystem calls during one loop reuse the same calculation.
  private LaunchingParameters latestParameters = null;
  private LaunchingParameters frozenParameters = null;
  private boolean passingModeLatched = false;
  private boolean passingYawTargetUseLeft = true;
  private boolean controllerPassingYawOnlyTargetEnabled = false;
  private int detailedLogCounter = 0;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching maps
  private static final InterpolatingTreeMap<Double, Double> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing maps
  private static final InterpolatingTreeMap<Double, Double> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Presets
  public static final double hubPresetDistance = 0.96;
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final double passingPresetDistance = 7.0;

  public static final LaunchPreset passingPreset;
  public static final LaunchPreset hubPreset;
  public static final LaunchPreset towerPreset;
  public static final LaunchPreset trenchPreset;
  public static final LaunchPreset outpostPreset;

  public static final LaunchPreset hoodMinPreset =
      new LaunchPreset(
          shooterCalcTunable(
              TABLE_KEY + "Presets/HoodMin/HoodAngle", Units.radiansToDegrees(0)),
          shooterCalcTunable(TABLE_KEY + "Presets/HoodMin/FlywheelSpeed", 1000));

  public static final LaunchPreset hoodMaxPreset =
      new LaunchPreset(
          shooterCalcTunable(
              TABLE_KEY + "Presets/HoodMax/HoodAngle", Units.radiansToDegrees(900)),
          shooterCalcTunable(TABLE_KEY + "Presets/HoodMax/FlywheelSpeed", 3000));

  // Passing target
  private static final double xPassTarget = Units.inchesToMeters(37);

  private static final LoggedTunableNumber passingModeXLineMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/ModeXLineMeters", FieldConstants.LinesVertical.hubCenter);

  private static final LoggedTunableNumber passingModeHysteresisMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/ModeHysteresisMeters", Units.inchesToMeters(12.0));

  private static final LoggedTunableNumber passingTargetXMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/TargetXMeters", xPassTarget);

  private static final LoggedTunableNumber passingTargetYOffsetMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/TargetYOffsetMeters", 0.0);

  private static final LoggedTunableNumber passingTargetMinYMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/TargetMinYMeters", Units.inchesToMeters(36.0));

  private static final LoggedTunableNumber passingTargetMaxYMeters =
      shooterCalcTunable(
          TABLE_KEY + "Passing/TargetMaxYMeters", FieldConstants.fieldWidth - Units.inchesToMeters(36.0));

  /** Enables the fixed left/right passing target for yaw aiming only. */
  private static final LoggedTunableBoolean passingYawOnlyFixedTargetEnabled =
      new LoggedTunableBoolean(
          TABLE_KEY + "Passing/YawOnlyFixedTargetEnabled",
          false,
          Constants.Tuning.SHOOTER_CALCULATION);

  /** X position of the blue-side fixed passing yaw target. */
  private static final LoggedTunableNumber passingYawTargetXMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/YawOnlyTargetXMeters", 5.5);

  /** Left-side blue-field Y position. The right-side target mirrors this across field center. */
  private static final LoggedTunableNumber passingYawTargetLeftYMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/YawOnlyTargetLeftYMeters", Units.feetToMeters(18.0));

  /** Hysteresis around field center when auto-selecting left vs. right passing yaw target. */
  private static final LoggedTunableNumber passingYawTargetSideHysteresisMeters =
      shooterCalcTunable(TABLE_KEY + "Passing/YawOnlyTargetSideHysteresisMeters", 0.25);

  private static final LoggedTunableNumber detailedLogPeriodLoops =
      shooterCalcTunable(TABLE_KEY + "Logging/DetailedPeriodLoops", 5.0);

  private static final double DEFAULT_NORMAL_SHOT_DISTANCE_FUDGE_METERS = 0.0;
  private static final double DEFAULT_PASSING_MAX_FLYWHEEL_VELOCITY_RPM = 3750.0;

  private static final LoggedTunableNumber normalShotDistanceFudgeStepMeters =
      shooterCalcTunable(TABLE_KEY + "Fudge/NormalShotDistanceStepMeters", 0.05);

  private static final LoggedTunableNumber passingMaxFlywheelVelocityStepRPM =
      shooterCalcTunable(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityStepRPM", 50.0);

  private double normalShotDistanceFudgeMeters = DEFAULT_NORMAL_SHOT_DISTANCE_FUDGE_METERS;
  private double passingMaxFlywheelVelocityRPM = DEFAULT_PASSING_MAX_FLYWHEEL_VELOCITY_RPM;

  // Boxes of bad
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  private static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(120),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  private static final LoggedTunableNumber hoodAngle1 =
      shooterCalcTunable("HoodAngleMap/0.0", 0.0);
  private static final LoggedTunableNumber hoodAngle2 =
      shooterCalcTunable("HoodAngleMap/1.46", 0.0);
  private static final LoggedTunableNumber hoodAngle3 =
      shooterCalcTunable("HoodAngleMap/1.73", 0.0);
  private static final LoggedTunableNumber hoodAngle4 =
      shooterCalcTunable("HoodAngleMap/2.18", 70.0);
  private static final LoggedTunableNumber hoodAngle5 =
      shooterCalcTunable("HoodAngleMap/2.47", 160.0);
  private static final LoggedTunableNumber hoodAngle6 =
      shooterCalcTunable("HoodAngleMap/2.70", 315.0);
  private static final LoggedTunableNumber hoodAngle7 =
      shooterCalcTunable("HoodAngleMap/2.94", 330.0);
  private static final LoggedTunableNumber hoodAngle8 =
      shooterCalcTunable("HoodAngleMap/3.48", 300.0);
  private static final LoggedTunableNumber hoodAngle9 =
      shooterCalcTunable("HoodAngleMap/3.92", 500.0);
  private static final LoggedTunableNumber hoodAngle10 =
      shooterCalcTunable("HoodAngleMap/4.35", 660.0);
  private static final LoggedTunableNumber hoodAngle11 =
      shooterCalcTunable("HoodAngleMap/4.84", 750.0);
  private static final LoggedTunableNumber hoodAngle12 =
      shooterCalcTunable("HoodAngleMap/5.46", 800.0);

  private static final LoggedTunableNumber flywheelSpeed1 =
      shooterCalcTunable("FlyWheelSpeedMap/0.0", 1750.0);
  private static final LoggedTunableNumber flywheelSpeed2 =
      shooterCalcTunable("FlyWheelSpeedMap/0.96", 1750.0);
  private static final LoggedTunableNumber flywheelSpeed3 =
      shooterCalcTunable("FlyWheelSpeedMap/1.46", 1800.0);
  private static final LoggedTunableNumber flywheelSpeed4 =
      shooterCalcTunable("FlyWheelSpeedMap/1.73", 1800.0);
  private static final LoggedTunableNumber flywheelSpeed5 =
      shooterCalcTunable("FlyWheelSpeedMap/2.18", 1800.0); //1900
  private static final LoggedTunableNumber flywheelSpeed6 =
      shooterCalcTunable("FlyWheelSpeedMap/2.47", 1850.0); //1950
  private static final LoggedTunableNumber flywheelSpeed7 =
      shooterCalcTunable("FlyWheelSpeedMap/2.70", 1950.0);
  private static final LoggedTunableNumber flywheelSpeed8 =
      shooterCalcTunable("FlyWheelSpeedMap/2.94", 2025.0);
  private static final LoggedTunableNumber flywheelSpeed9 =
      shooterCalcTunable("FlyWheelSpeedMap/3.48", 2200.0);
  private static final LoggedTunableNumber flywheelSpeed10 =
      shooterCalcTunable("FlyWheelSpeedMap/3.92", 2300.0);
  private static final LoggedTunableNumber flywheelSpeed11 =
      shooterCalcTunable("FlyWheelSpeedMap/4.35", 2600.0);
  private static final LoggedTunableNumber flywheelSpeed12 =
      shooterCalcTunable("FlyWheelSpeedMap/4.84", 2700.0);
  private static final LoggedTunableNumber flywheelSpeed13 =
      shooterCalcTunable("FlyWheelSpeedMap/5.46", 2800.0);

  private static final LoggedTunableNumber passingHoodAngle1 =
      shooterCalcTunable("PassingHoodMap/3.4", 800.0);
  private static final LoggedTunableNumber passingHoodAngle2 =
      shooterCalcTunable("PassingHoodMap/4.275", 800.0);
  private static final LoggedTunableNumber passingHoodAngle3 =
      shooterCalcTunable("PassingHoodMap/5.46", 800.0);
  private static final LoggedTunableNumber passingHoodAngle4 =
      shooterCalcTunable("PassingHoodMap/6.62", 800.0);
  private static final LoggedTunableNumber passingHoodAngle5 =
      shooterCalcTunable("PassingHoodMap/7.8", 800.0);
  private static final LoggedTunableNumber passingHoodAngle6 =
      shooterCalcTunable("PassingHoodMap/8.67", 800.0);
  private static final LoggedTunableNumber passingHoodAngle7 =
      shooterCalcTunable("PassingHoodMap/9.65", 800.0);
  private static final LoggedTunableNumber passingHoodAngle8 =
      shooterCalcTunable("PassingHoodMap/10.16", 800.0);
  private static final LoggedTunableNumber passingHoodAngle9 =
      shooterCalcTunable("PassingHoodMap/15.00", 800.0);
  private static final LoggedTunableNumber passingHoodAngleLast =
      shooterCalcTunable("PassingHoodMap/17.16", 850.0);

  private static final LoggedTunableNumber passingFlywheelSpeed1 =
      shooterCalcTunable("PassingFlywheelMap/3.4", 2300.0);
  private static final LoggedTunableNumber passingFlywheelSpeed2 =
      shooterCalcTunable("PassingFlywheelMap/4.275", 2300.0);
  private static final LoggedTunableNumber passingFlywheelSpeed3 =
      shooterCalcTunable("PassingFlywheelMap/5.46", 2500.0);
  private static final LoggedTunableNumber passingFlywheelSpeed4 =
      shooterCalcTunable("PassingFlywheelMap/6.62", 2600.0);
  private static final LoggedTunableNumber passingFlywheelSpeed5 =
      shooterCalcTunable("PassingFlywheelMap/7.8", 2600.0);
  private static final LoggedTunableNumber passingFlywheelSpeed6 =
      shooterCalcTunable("PassingFlywheelMap/8.67", 2750.0);
  private static final LoggedTunableNumber passingFlywheelSpeed7 =
      shooterCalcTunable("PassingFlywheelMap/9.65", 3000.0);
  private static final LoggedTunableNumber passingFlywheelSpeed8 =
      shooterCalcTunable("PassingFlywheelMap/10.16", 3750.0);
  private static final LoggedTunableNumber passingFlywheelSpeedLast =
      shooterCalcTunable("PassingFlywheelMap/17.16", 3750.0);

  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 5.4;
    passingMaxDistance = 17.16;
    phaseDelay = 0.03;

    loadHoodAngleMap();
    loadFlywheelSpeedMap();
    loadTimeOfFlightMap();
    loadPassingHoodAngleMap();
    loadPassingFlywheelSpeedMap();
    loadPassingTimeOfFlightMap();

    passingPreset =
        new LaunchPreset(
            shooterCalcTunable(
                TABLE_KEY + "Presets/Passing/HoodAngle",
                hoodAngleMap.get(passingPresetDistance)),
            shooterCalcTunable(
                TABLE_KEY + "Presets/Passing/FlywheelSpeed",
                flywheelSpeedMap.get(passingPresetDistance)));

    hubPreset =
        new LaunchPreset(
            shooterCalcTunable(
                TABLE_KEY + "Presets/Hub/HoodAngle", hoodAngleMap.get(hubPresetDistance)),
            shooterCalcTunable(
                TABLE_KEY + "Presets/Hub/FlywheelSpeed",
                flywheelSpeedMap.get(hubPresetDistance)));

    towerPreset =
        new LaunchPreset(
            shooterCalcTunable(
                TABLE_KEY + "Presets/Tower/HoodAngle", hoodAngleMap.get(towerPresetDistance)),
            shooterCalcTunable(
                TABLE_KEY + "Presets/Tower/FlywheelSpeed",
                flywheelSpeedMap.get(towerPresetDistance)));

    trenchPreset =
        new LaunchPreset(
            shooterCalcTunable(
                TABLE_KEY + "Presets/Trench/HoodAngle", hoodAngleMap.get(trenchPresetDistance)),
            shooterCalcTunable(
                TABLE_KEY + "Presets/Trench/FlywheelSpeed",
                flywheelSpeedMap.get(trenchPresetDistance)));

    outpostPreset =
        new LaunchPreset(
            shooterCalcTunable(
                TABLE_KEY + "Presets/Outpost/HoodAngle", hoodAngleMap.get(outpostPresetDistance)),
            shooterCalcTunable(
                TABLE_KEY + "Presets/Outpost/FlywheelSpeed",
                flywheelSpeedMap.get(outpostPresetDistance)));
  }

  /** Returns the minimum normal shot time of flight. */
  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  /** Returns the maximum normal shot time of flight. */
  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  /**
   * Updates tunable interpolation maps when the dashboard update flags are toggled.
   *
   * <p>This is only called while tuning mode is enabled.
   */
  public void updateMaps() {
    if (SmartDashboard.getBoolean(updateHoodMapKey, true)) {
      SmartDashboard.putBoolean(updateHoodMapKey, false);
      loadHoodAngleMap();
    }

    if (SmartDashboard.getBoolean(updateFlywheelMapKey, true)) {
      SmartDashboard.putBoolean(updateFlywheelMapKey, false);
      flywheelSpeedMap.clear();
      loadFlywheelSpeedMap();
    }

    if (SmartDashboard.getBoolean(updatePassingHoodMapKey, true)) {
      SmartDashboard.putBoolean(updatePassingHoodMapKey, false);
      loadPassingHoodAngleMap();
    }

    if (SmartDashboard.getBoolean(updatePassingFlywheelMapKey, true)) {
      SmartDashboard.putBoolean(updatePassingFlywheelMapKey, false);
      passingFlywheelSpeedMap.clear();
      loadPassingFlywheelSpeedMap();
    }
  }

  /**
   * Calculates and returns current shooter launch parameters.
   *
   * <p>The result is cached for the rest of the loop and should be cleared once per loop by calling
   * {@link #clearLaunchingParameters()}.
   *
   * @return current launch parameters
   */
  public LaunchingParameters getParameters() {
    if (frozenParameters != null) {
      return frozenParameters;
    }

    if (latestParameters != null) {
      return latestParameters;
    }

    if (Constants.Tuning.SHOOTER_CALCULATION) {
      updateMaps();
    }

    Pose2d estimatedPose = getPhaseDelayedRobotPose();
    boolean passing = shouldUsePassingMode(estimatedPose);
    Translation2d distanceTarget =
        passing
            ? getPassingTarget(estimatedPose)
            : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Translation2d aimTarget = passing ? getPassingAimTarget(estimatedPose, distanceTarget) : distanceTarget;

    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher.toTransform2d());
    double rawLauncherToTargetDistance = distanceTarget.getDistance(launcherPosition.getTranslation());

    ChassisSpeeds robotVelocity = RobotState.getInstance().getDesiredFieldRelativeSpeeds();
    Rotation2d robotAngle = RobotState.getInstance().getYawForVision();

    // Use full velocity for logging / distance if you want, but not for aim heading compensation.
    // Translational velocity is the part that matters for strafe SOTM lead.
    // Robot omega during aiming creates a feedback loop because the target moves while the robot turns.
    ChassisSpeeds aimRobotVelocity =
        new ChassisSpeeds(
            robotVelocity.vxMetersPerSecond,
            robotVelocity.vyMetersPerSecond,
            0.0);

    ChassisSpeeds launcherVelocity = getFieldRelativeLauncherVelocity(aimRobotVelocity, robotAngle);

    ChassisSpeeds velocityDerivedAcceleration =
        RobotState.getInstance().getMeasuredFieldRelativeAcceleration();
    ChassisSpeeds imuAcceleration =
        RobotState.getInstance().getMeasuredFieldRelativeImuAcceleration();
    ChassisSpeeds blendedRobotAcceleration =
        getBlendedFieldRelativeAcceleration(velocityDerivedAcceleration, imuAcceleration);
    ChassisSpeeds launcherAcceleration =
        getFieldRelativeLauncherAcceleration(blendedRobotAcceleration, robotVelocity, robotAngle);

    double rawTimeOfFlight = getTimeOfFlight(rawLauncherToTargetDistance, passing);
    Rotation2d currentPoseDriveAngle = getDriveAngleWithLauncherOffset(estimatedPose, aimTarget);

    Pose2d distanceLookaheadPose =
        getMotionCompensatedPose(
            launcherPosition,
            launcherVelocity,
            launcherAcceleration,
            distanceTarget,
            passing,
            distanceVelocityCompensationScalar.get(),
            distanceAccelerationCompensationScalar.get());

    double compensatedDistance = distanceTarget.getDistance(distanceLookaheadPose.getTranslation());
    double compensatedTimeOfFlight = getTimeOfFlight(compensatedDistance, passing);

    Pose2d aimLookaheadPose =
        offsetPoseByMotion(
            launcherPosition,
            launcherVelocity,
            launcherAcceleration,
            compensatedTimeOfFlight,
            aimVelocityCompensationScalar.get(),
            aimAccelerationCompensationScalar.get());

    Pose2d aimLookaheadRobotPose =
        aimLookaheadPose.transformBy(robotToLauncher.toTransform2d().inverse());

    boolean simpleTargetShiftActive = simpleTargetShiftEnabled.getAsBoolean();
    Translation2d simpleTargetShift =
        new Translation2d(
            launcherVelocity.vxMetersPerSecond
                * compensatedTimeOfFlight
                * simpleTargetShiftScalar.getAsDouble(),
            launcherVelocity.vyMetersPerSecond
                * compensatedTimeOfFlight
                * simpleTargetShiftScalar.getAsDouble());
    Translation2d activeAimTarget =
        simpleTargetShiftActive ? aimTarget.plus(simpleTargetShift) : aimTarget;
    Pose2d activeAimRobotPose = simpleTargetShiftActive ? estimatedPose : aimLookaheadRobotPose;
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(activeAimRobotPose, activeAimTarget);

    double activeMapDistance =
        passing ? compensatedDistance : compensatedDistance + normalShotDistanceFudgeMeters;

    double hoodAngle =
        passing ? passingHoodAngleMap.get(activeMapDistance) : hoodAngleMap.get(activeMapDistance);
    double hoodAngleWithOffset = hoodAngle + Units.degreesToRotations(hoodAngleOffsetDeg);

    if (lastDriveAngle == null) {
      lastDriveAngle = driveAngle;
    }
    if (Double.isNaN(lastHoodAngle)) {
      lastHoodAngle = hoodAngle;
    }

    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;

    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    boolean outsideOfBadBoxes = isOutsideBadBoxes(estimatedPose);
    double uncappedFlywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(activeMapDistance)
            : flywheelSpeedMap.get(activeMapDistance);
    double flywheelVelocity =
        passing
            ? Math.min(uncappedFlywheelVelocity, passingMaxFlywheelVelocityRPM)
            : uncappedFlywheelVelocity;

    Logger.recordOutput(TABLE_KEY + "Fudge/NormalShotDistanceMeters", normalShotDistanceFudgeMeters);
    Logger.recordOutput(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityRPM", passingMaxFlywheelVelocityRPM);
    Logger.recordOutput(TABLE_KEY + "Fudge/ActiveMapDistance", activeMapDistance);
    Logger.recordOutput(TABLE_KEY + "Fudge/UnfudgedCompensatedDistance", compensatedDistance);
    Logger.recordOutput(TABLE_KEY + "Fudge/UncappedFlywheelVelocity", uncappedFlywheelVelocity);
    Logger.recordOutput(TABLE_KEY + "Fudge/ActiveFlywheelVelocity", flywheelVelocity);

    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && compensatedDistance >= (passing ? passingMinDistance : minDistance)
                && compensatedDistance <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngleWithOffset,
            hoodVelocity,
            flywheelVelocity,
            activeMapDistance,
            rawLauncherToTargetDistance,
            compensatedTimeOfFlight,
            passing);

    logCalculation(
        passing,
        distanceTarget,
        activeAimTarget,
        launcherPosition,
        launcherVelocity,
        launcherAcceleration,
        velocityDerivedAcceleration,
        imuAcceleration,
        blendedRobotAcceleration,
        rawLauncherToTargetDistance,
        compensatedDistance,
        rawTimeOfFlight,
        compensatedTimeOfFlight,
        aimLookaheadPose,
        aimLookaheadRobotPose,
        distanceLookaheadPose,
        hoodAngleWithOffset,
        hoodVelocity,
        flywheelVelocity,
        currentPoseDriveAngle,
        true);

    return latestParameters;
  }

  /**
   * Returns the naive time of flight for a distance.
   *
   * @param distance distance in meters
   * @return time of flight in seconds
   */
  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  /** Freezes the current calculated launch parameters until {@link #unfreezeLaunchingParameters()} is called. */
  public void freezeLaunchingParameters() {
    frozenParameters = getParameters();
    Logger.recordOutput(TABLE_KEY + "Freeze/Active", true);
  }

  /** Allows launch parameters to update normally again. */
  public void unfreezeLaunchingParameters() {
    frozenParameters = null;
    Logger.recordOutput(TABLE_KEY + "Freeze/Active", false);
  }

  /** Clears the cached launch parameters so they are recalculated on the next request. */
  public void clearLaunchingParameters() {
    if (frozenParameters == null) {
      latestParameters = null;
    }
  }

  /**
   * Returns the passing target used for hood/RPM distance tracking.
   *
   * <p>This target intentionally keeps the old dynamic-Y behavior so passing hood and flywheel maps
   * keep using the same distance basis they were tuned against.
   *
   * @return passing distance target translation
   */
  public Translation2d getPassingTarget() {
    return getPassingTarget(RobotState.getInstance().getRobotPoseField());
  }

  private Translation2d getPassingTarget(Pose2d robotPose) {
    double flippedY = AllianceFlipUtil.apply(robotPose).getY();
    double targetY =
        MathUtil.clamp(
            flippedY + passingTargetYOffsetMeters.get(),
            passingTargetMinYMeters.get(),
            passingTargetMaxYMeters.get());

    return AllianceFlipUtil.apply(new Translation2d(passingTargetXMeters.get(), targetY));
  }

  private Translation2d getPassingAimTarget(Pose2d robotPose, Translation2d distanceTarget) {
    if (!isPassingYawOnlyFixedTargetEnabled()) {
      return distanceTarget;
    }

    return getAutoLeftRightPassingYawTarget(robotPose);
  }

  private Translation2d getAutoLeftRightPassingYawTarget(Pose2d robotPose) {
    double flippedY = AllianceFlipUtil.apply(robotPose).getY();
    double fieldCenterY = FieldConstants.fieldWidth / 2.0;
    double sideHysteresis = Math.max(0.0, passingYawTargetSideHysteresisMeters.get());

    if (passingYawTargetUseLeft) {
      passingYawTargetUseLeft = flippedY >= fieldCenterY - sideHysteresis;
    } else {
      passingYawTargetUseLeft = flippedY >= fieldCenterY + sideHysteresis;
    }

    double leftTargetY =
        MathUtil.clamp(passingYawTargetLeftYMeters.get(), 0.0, FieldConstants.fieldWidth);
    double rightTargetY = FieldConstants.fieldWidth - leftTargetY;
    double targetY = passingYawTargetUseLeft ? leftTargetY : rightTargetY;

    return AllianceFlipUtil.apply(new Translation2d(passingYawTargetXMeters.get(), targetY));
  }

  public void togglePassingYawOnlyFixedTarget() {
    setPassingYawOnlyFixedTargetEnabled(!controllerPassingYawOnlyTargetEnabled);
  }

  public void setPassingYawOnlyFixedTargetEnabled(boolean enabled) {
    controllerPassingYawOnlyTargetEnabled = enabled;
    Logger.recordOutput(TABLE_KEY + "Passing/YawOnlyControllerEnabled", enabled);
  }

  public boolean isPassingYawOnlyFixedTargetEnabled() {
    return passingYawOnlyFixedTargetEnabled.getAsBoolean() || controllerPassingYawOnlyTargetEnabled;
  }

  /** Returns whether the current pose should use passing maps, with hysteresis around the X line. */
  private boolean shouldUsePassingMode(Pose2d estimatedPose) {
    double flippedX = AllianceFlipUtil.applyX(estimatedPose.getX());
    double threshold = passingModeXLineMeters.get();
    double hysteresis = Math.max(0.0, passingModeHysteresisMeters.get());

    if (passingModeLatched) {
      passingModeLatched = flippedX > threshold - hysteresis;
    } else {
      passingModeLatched = flippedX > threshold + hysteresis;
    }

    return passingModeLatched;
  }


  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation translation of the center of the robot
   * @param forceBlue true to always use the blue hub target
   * @return target pose for the aimed robot
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
  }

  /**
   * Adjusts the hood angle offset up or down.
   *
   * @param incrementDegrees offset change in degrees
   */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }

  /** Increases the normal-shot distance fudge used for hood and flywheel map lookup. */
  public void increaseNormalShotDistanceFudge() {
    normalShotDistanceFudgeMeters += Math.abs(normalShotDistanceFudgeStepMeters.getAsDouble());
    Logger.recordOutput(TABLE_KEY + "Fudge/NormalShotDistanceMeters", normalShotDistanceFudgeMeters);
  }

  /** Decreases the normal-shot distance fudge used for hood and flywheel map lookup. */
  public void decreaseNormalShotDistanceFudge() {
    normalShotDistanceFudgeMeters -= Math.abs(normalShotDistanceFudgeStepMeters.getAsDouble());
    Logger.recordOutput(TABLE_KEY + "Fudge/NormalShotDistanceMeters", normalShotDistanceFudgeMeters);
  }

  /** Increases the maximum allowed passing flywheel velocity. */
  public void increasePassingMaxFlywheelVelocity() {
    passingMaxFlywheelVelocityRPM =
        MathUtil.clamp(
            passingMaxFlywheelVelocityRPM + Math.abs(passingMaxFlywheelVelocityStepRPM.getAsDouble()),
            0.0,
            MAX_FLYWHEEL_CALCULATED_RPM);
    Logger.recordOutput(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityRPM", passingMaxFlywheelVelocityRPM);
  }

  /** Decreases the maximum allowed passing flywheel velocity. */
  public void decreasePassingMaxFlywheelVelocity() {
    passingMaxFlywheelVelocityRPM =
        MathUtil.clamp(
            passingMaxFlywheelVelocityRPM - Math.abs(passingMaxFlywheelVelocityStepRPM.getAsDouble()),
            0.0,
            MAX_FLYWHEEL_CALCULATED_RPM);
    Logger.recordOutput(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityRPM", passingMaxFlywheelVelocityRPM);
  }

  /** Restores the normal-shot distance fudge and passing flywheel cap to their defaults. */
  public void resetShotFudgeFactors() {
    normalShotDistanceFudgeMeters = DEFAULT_NORMAL_SHOT_DISTANCE_FUDGE_METERS;
    passingMaxFlywheelVelocityRPM = DEFAULT_PASSING_MAX_FLYWHEEL_VELOCITY_RPM;
    Logger.recordOutput(TABLE_KEY + "Fudge/NormalShotDistanceMeters", normalShotDistanceFudgeMeters);
    Logger.recordOutput(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityRPM", passingMaxFlywheelVelocityRPM);
  }

  private Pose2d getPhaseDelayedRobotPose() {
    Pose2d estimatedPose = RobotState.getInstance().getRobotPoseField();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getMeasuredRobotRelativeSpeeds();

    return estimatedPose.exp(
        new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
  }

  private ChassisSpeeds getFieldRelativeLauncherVelocity(
      ChassisSpeeds robotVelocity, Rotation2d robotAngle) {
    return GeomUtil.transformVelocity(
        robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);
  }

  private ChassisSpeeds getBlendedFieldRelativeAcceleration(
      ChassisSpeeds velocityDerivedAcceleration, ChassisSpeeds imuAcceleration) {
    double velocityWeight = velocityDerivedAccelerationWeight.get();
    double imuWeight = imuAccelerationWeight.get();

    return new ChassisSpeeds(
        velocityDerivedAcceleration.vxMetersPerSecond * velocityWeight
            + imuAcceleration.vxMetersPerSecond * imuWeight,
        velocityDerivedAcceleration.vyMetersPerSecond * velocityWeight
            + imuAcceleration.vyMetersPerSecond * imuWeight,
        velocityDerivedAcceleration.omegaRadiansPerSecond * velocityWeight
            + imuAcceleration.omegaRadiansPerSecond * imuWeight);
  }

  private ChassisSpeeds getFieldRelativeLauncherAcceleration(
      ChassisSpeeds robotAcceleration, ChassisSpeeds robotVelocity, Rotation2d robotAngle) {
    Translation2d launcherOffsetField =
        robotToLauncher.getTranslation().toTranslation2d().rotateBy(robotAngle);

    double rx = launcherOffsetField.getX();
    double ry = launcherOffsetField.getY();

    double omega = robotVelocity.omegaRadiansPerSecond;
    double alpha = robotAcceleration.omegaRadiansPerSecond;

    double launcherAccelX =
        robotAcceleration.vxMetersPerSecond - alpha * ry - omega * omega * rx;
    double launcherAccelY =
        robotAcceleration.vyMetersPerSecond + alpha * rx - omega * omega * ry;

    return new ChassisSpeeds(launcherAccelX, launcherAccelY, alpha);
  }

  private Pose2d getMotionCompensatedPose(
      Pose2d launcherPosition,
      ChassisSpeeds launcherVelocity,
      ChassisSpeeds launcherAcceleration,
      Translation2d target,
      boolean passing,
      double velocityScalar,
      double accelerationScalar) {
    double lookaheadDistance = target.getDistance(launcherPosition.getTranslation());
    Pose2d lookaheadPose = launcherPosition;

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double timeOfFlight = getTimeOfFlight(lookaheadDistance, passing);
      lookaheadPose =
          offsetPoseByMotion(
              launcherPosition,
              launcherVelocity,
              launcherAcceleration,
              timeOfFlight,
              velocityScalar,
              accelerationScalar);
      lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    return lookaheadPose;
  }

  private Pose2d offsetPoseByMotion(
      Pose2d pose,
      ChassisSpeeds velocity,
      ChassisSpeeds acceleration,
      double timeOfFlight,
      double velocityScalar,
      double accelerationScalar) {
    MotionOffset offset =
        calculateMotionOffset(velocity, acceleration, timeOfFlight, velocityScalar, accelerationScalar);

    return new Pose2d(
        pose.getTranslation().plus(new Translation2d(offset.totalX(), offset.totalY())),
        pose.getRotation());
  }

  private MotionOffset calculateMotionOffset(
      ChassisSpeeds velocity,
      ChassisSpeeds acceleration,
      double timeOfFlight,
      double velocityScalar,
      double accelerationScalar) {
    double velocityOffsetX = velocity.vxMetersPerSecond * timeOfFlight * velocityScalar;
    double velocityOffsetY = velocity.vyMetersPerSecond * timeOfFlight * velocityScalar;

    double accelerationOffsetX =
        0.5 * acceleration.vxMetersPerSecond * timeOfFlight * timeOfFlight * accelerationScalar;
    double accelerationOffsetY =
        0.5 * acceleration.vyMetersPerSecond * timeOfFlight * timeOfFlight * accelerationScalar;

    return new MotionOffset(velocityOffsetX, velocityOffsetY, accelerationOffsetX, accelerationOffsetY);
  }

  private double getTimeOfFlight(double distance, boolean passing) {
    return passing ? passingTimeOfFlightMap.get(distance) : timeOfFlightMap.get(distance);
  }

  private boolean isOutsideBadBoxes(Pose2d estimatedPose) {
    Pose2d flippedPose = AllianceFlipUtil.apply(estimatedPose);

    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());

    return !(insideTowerBadBox || behindNearHub || behindFarHub);
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));

    return fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
  }

  /** Returns true when the expensive shooter-calculation debug log should publish this loop. */
  private boolean shouldLogDetailedCalculation() {
    if (!Constants.Tuning.SHOOTER_CALCULATION && !Constants.Tuning.SOTM) {
      return false;
    }

    int detailedPeriodLoops = Math.max(1, (int) Math.round(detailedLogPeriodLoops.get()));
    return detailedLogCounter++ % detailedPeriodLoops == 0;
  }

  private void logCalculation(
      boolean passing,
      Translation2d target,
      Translation2d aimTarget,
      Pose2d launcherPosition,
      ChassisSpeeds launcherVelocity,
      ChassisSpeeds launcherAcceleration,
      ChassisSpeeds velocityDerivedAcceleration,
      ChassisSpeeds imuAcceleration,
      ChassisSpeeds blendedRobotAcceleration,
      double rawDistance,
      double compensatedDistance,
      double rawTimeOfFlight,
      double compensatedTimeOfFlight,
      Pose2d aimLookaheadPose,
      Pose2d aimLookaheadRobotPose,
      Pose2d distanceLookaheadPose,
      double hoodAngle,
      double hoodVelocity,
      double flywheelVelocity,
      Rotation2d currentPoseDriveAngle,
      boolean lookaheadAllowed) {
    if (!shouldLogDetailedCalculation()) {
      return;
    }

    int detailedPeriodLoops = Math.max(1, (int) Math.round(detailedLogPeriodLoops.get()));

    Translation2d launcherToTarget = target.minus(launcherPosition.getTranslation());
    Translation2d unitToTarget = launcherToTarget.div(launcherToTarget.getNorm());

    double radialVelocity =
        launcherVelocity.vxMetersPerSecond * unitToTarget.getX()
            + launcherVelocity.vyMetersPerSecond * unitToTarget.getY();
    double tangentialVelocity =
        launcherVelocity.vxMetersPerSecond * -unitToTarget.getY()
            + launcherVelocity.vyMetersPerSecond * unitToTarget.getX();

    double radialAcceleration =
        launcherAcceleration.vxMetersPerSecond * unitToTarget.getX()
            + launcherAcceleration.vyMetersPerSecond * unitToTarget.getY();
    double tangentialAcceleration =
        launcherAcceleration.vxMetersPerSecond * -unitToTarget.getY()
            + launcherAcceleration.vyMetersPerSecond * unitToTarget.getX();

    MotionOffset aimOffset =
        calculateMotionOffset(
            launcherVelocity,
            launcherAcceleration,
            compensatedTimeOfFlight,
            aimVelocityCompensationScalar.get(),
            aimAccelerationCompensationScalar.get());

    MotionOffset distanceOffset =
        calculateMotionOffset(
            launcherVelocity,
            launcherAcceleration,
            compensatedTimeOfFlight,
            distanceVelocityCompensationScalar.get(),
            distanceAccelerationCompensationScalar.get());

    Logger.recordOutput(TABLE_KEY + "Passing", passing);
    Logger.recordOutput(TABLE_KEY + "Passing/ModeLatched", passingModeLatched);
    Logger.recordOutput(TABLE_KEY + "Passing/YawOnlyFixedTargetEnabled", isPassingYawOnlyFixedTargetEnabled());
    Logger.recordOutput(TABLE_KEY + "Passing/YawOnlyDashboardEnabled", passingYawOnlyFixedTargetEnabled.getAsBoolean());
    Logger.recordOutput(TABLE_KEY + "Passing/YawOnlyControllerEnabled", controllerPassingYawOnlyTargetEnabled);
    Logger.recordOutput(TABLE_KEY + "Passing/YawOnlyUsingLeft", passingYawTargetUseLeft);
    Logger.recordOutput(TABLE_KEY + "SOTM/LookaheadAllowedCached", lookaheadAllowed);
    Logger.recordOutput(
        TABLE_KEY + "SOTM/TranslationSpeedMetersPerSec",
        Math.hypot(launcherVelocity.vxMetersPerSecond, launcherVelocity.vyMetersPerSecond));
    Logger.recordOutput(TABLE_KEY + "SOTM/LauncherOmegaRadPerSec", launcherVelocity.omegaRadiansPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "SOTM/CurrentAngleErrorDeg",
        currentPoseDriveAngle.minus(RobotState.getInstance().getYawForVision()).getDegrees());
    Logger.recordOutput(TABLE_KEY + "DriveAngle/CurrentPose", currentPoseDriveAngle);
    Logger.recordOutput(TABLE_KEY + "SimpleTargetShift/Enabled", simpleTargetShiftEnabled.getAsBoolean());
    Logger.recordOutput(TABLE_KEY + "SimpleTargetShift/Scalar", simpleTargetShiftScalar.getAsDouble());
    Logger.recordOutput(
        TABLE_KEY + "SimpleTargetShift/Applied", simpleTargetShiftEnabled.getAsBoolean());
    Logger.recordOutput(TABLE_KEY + "Logging/DetailedPeriodLoopsActive", detailedPeriodLoops);

    Logger.recordOutput(TABLE_KEY + "Target/Distance", target);
    Logger.recordOutput(TABLE_KEY + "Target/Aim", aimTarget);
    Logger.recordOutput(TABLE_KEY + "Target/DistancePose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput(TABLE_KEY + "Target/AimPose", new Pose2d(aimTarget, Rotation2d.kZero));

    Logger.recordOutput(TABLE_KEY + "HoodAngle", hoodAngle);
    Logger.recordOutput(TABLE_KEY + "HoodVelocity", hoodVelocity);
    Logger.recordOutput(TABLE_KEY + "FlywheelVelocity", flywheelVelocity);

    Logger.recordOutput(TABLE_KEY + "Distance/Raw", rawDistance);
    Logger.recordOutput(TABLE_KEY + "Distance/Compensated", compensatedDistance);
    Logger.recordOutput(TABLE_KEY + "Distance/Delta", compensatedDistance - rawDistance);
    Logger.recordOutput(TABLE_KEY + "Fudge/NormalShotDistanceMeters", normalShotDistanceFudgeMeters);
    Logger.recordOutput(TABLE_KEY + "Fudge/PassingMaxFlywheelVelocityRPM", passingMaxFlywheelVelocityRPM);

    Logger.recordOutput(TABLE_KEY + "TimeOfFlight/Raw", rawTimeOfFlight);
    Logger.recordOutput(TABLE_KEY + "TimeOfFlight/Compensated", compensatedTimeOfFlight);

    Logger.recordOutput(TABLE_KEY + "Velocity/LauncherFieldX", launcherVelocity.vxMetersPerSecond);
    Logger.recordOutput(TABLE_KEY + "Velocity/LauncherFieldY", launcherVelocity.vyMetersPerSecond);
    Logger.recordOutput(TABLE_KEY + "Velocity/LauncherOmega", launcherVelocity.omegaRadiansPerSecond);
    Logger.recordOutput(TABLE_KEY + "Velocity/Radial", radialVelocity);
    Logger.recordOutput(TABLE_KEY + "Velocity/Tangential", tangentialVelocity);

    Logger.recordOutput(
        TABLE_KEY + "Acceleration/VelocityDerivedFieldX",
        velocityDerivedAcceleration.vxMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/VelocityDerivedFieldY",
        velocityDerivedAcceleration.vyMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/VelocityDerivedOmega",
        velocityDerivedAcceleration.omegaRadiansPerSecond);
    Logger.recordOutput(TABLE_KEY + "Acceleration/ImuFieldX", imuAcceleration.vxMetersPerSecond);
    Logger.recordOutput(TABLE_KEY + "Acceleration/ImuFieldY", imuAcceleration.vyMetersPerSecond);
    Logger.recordOutput(TABLE_KEY + "Acceleration/ImuOmega", imuAcceleration.omegaRadiansPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/BlendedRobotFieldX",
        blendedRobotAcceleration.vxMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/BlendedRobotFieldY",
        blendedRobotAcceleration.vyMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/BlendedRobotOmega",
        blendedRobotAcceleration.omegaRadiansPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/LauncherFieldX",
        launcherAcceleration.vxMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/LauncherFieldY",
        launcherAcceleration.vyMetersPerSecond);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/LauncherAlpha",
        launcherAcceleration.omegaRadiansPerSecond);
    Logger.recordOutput(TABLE_KEY + "Acceleration/Radial", radialAcceleration);
    Logger.recordOutput(TABLE_KEY + "Acceleration/Tangential", tangentialAcceleration);
    Logger.recordOutput(
        TABLE_KEY + "Acceleration/VelocityDerivedWeight",
        velocityDerivedAccelerationWeight.get());
    Logger.recordOutput(TABLE_KEY + "Acceleration/ImuWeight", imuAccelerationWeight.get());

    Logger.recordOutput(TABLE_KEY + "Lookahead/AimVelocityScalar", aimVelocityCompensationScalar.get());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceVelocityScalar", distanceVelocityCompensationScalar.get());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/AimAccelerationScalar", aimAccelerationCompensationScalar.get());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceAccelerationScalar",
        distanceAccelerationCompensationScalar.get());

    Logger.recordOutput(TABLE_KEY + "Lookahead/AimVelocityOffsetX", aimOffset.velocityX());
    Logger.recordOutput(TABLE_KEY + "Lookahead/AimVelocityOffsetY", aimOffset.velocityY());
    Logger.recordOutput(TABLE_KEY + "Lookahead/AimAccelerationOffsetX", aimOffset.accelerationX());
    Logger.recordOutput(TABLE_KEY + "Lookahead/AimAccelerationOffsetY", aimOffset.accelerationY());
    Logger.recordOutput(TABLE_KEY + "Lookahead/AimTotalOffsetX", aimOffset.totalX());
    Logger.recordOutput(TABLE_KEY + "Lookahead/AimTotalOffsetY", aimOffset.totalY());

    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceVelocityOffsetX", distanceOffset.velocityX());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceVelocityOffsetY", distanceOffset.velocityY());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceAccelerationOffsetX", distanceOffset.accelerationX());
    Logger.recordOutput(
        TABLE_KEY + "Lookahead/DistanceAccelerationOffsetY", distanceOffset.accelerationY());
    Logger.recordOutput(TABLE_KEY + "Lookahead/DistanceTotalOffsetX", distanceOffset.totalX());
    Logger.recordOutput(TABLE_KEY + "Lookahead/DistanceTotalOffsetY", distanceOffset.totalY());

    Logger.recordOutput(TABLE_KEY + "Lookahead/AimPose", aimLookaheadRobotPose);
    Logger.recordOutput(TABLE_KEY + "Lookahead/DistancePose", distanceLookaheadPose);
  }

  private static void loadHoodAngleMap() {
    hoodAngleMap.put(0.0, hoodAngle1.get());
    hoodAngleMap.put(1.46, hoodAngle2.get());
    hoodAngleMap.put(1.73, hoodAngle3.get());
    hoodAngleMap.put(2.18, hoodAngle4.get());
    hoodAngleMap.put(2.47, hoodAngle5.get());
    hoodAngleMap.put(2.70, hoodAngle6.get());
    hoodAngleMap.put(2.94, hoodAngle7.get());
    hoodAngleMap.put(3.48, hoodAngle8.get());
    hoodAngleMap.put(3.92, hoodAngle9.get());
    hoodAngleMap.put(4.35, hoodAngle10.get());
    hoodAngleMap.put(4.84, hoodAngle11.get());
    hoodAngleMap.put(5.46, hoodAngle12.get());
  }

  private static void loadFlywheelSpeedMap() {
    flywheelSpeedMap.put(0.0, flywheelSpeed1.get());
    flywheelSpeedMap.put(0.96, flywheelSpeed2.get());
    flywheelSpeedMap.put(1.46, flywheelSpeed3.get());
    flywheelSpeedMap.put(1.73, flywheelSpeed4.get());
    flywheelSpeedMap.put(2.18, flywheelSpeed5.get());
    flywheelSpeedMap.put(2.47, flywheelSpeed6.get());
    flywheelSpeedMap.put(2.70, flywheelSpeed7.get());
    flywheelSpeedMap.put(2.94, flywheelSpeed8.get());
    flywheelSpeedMap.put(3.48, flywheelSpeed9.get());
    flywheelSpeedMap.put(3.92, flywheelSpeed10.get());
    flywheelSpeedMap.put(4.35, flywheelSpeed11.get());
    flywheelSpeedMap.put(4.84, flywheelSpeed12.get());
    flywheelSpeedMap.put(5.46, flywheelSpeed13.get());
  }

  private static void loadTimeOfFlightMap() {
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  private static void loadPassingHoodAngleMap() {
    passingHoodAngleMap.put(3.40, passingHoodAngle1.get());
    passingHoodAngleMap.put(4.275, passingHoodAngle2.get());
    passingHoodAngleMap.put(5.46, passingHoodAngle3.get());
    passingHoodAngleMap.put(6.62, passingHoodAngle4.get());
    passingHoodAngleMap.put(7.80, passingHoodAngle5.get());
    passingHoodAngleMap.put(8.67, passingHoodAngle6.get());
    passingHoodAngleMap.put(9.65, passingHoodAngle7.get());
    passingHoodAngleMap.put(10.16, passingHoodAngle8.get());
    passingHoodAngleMap.put(15.00, passingHoodAngle9.get());
    passingHoodAngleMap.put(17.16, passingHoodAngleLast.get());
  }

  private static void loadPassingFlywheelSpeedMap() {
    passingFlywheelSpeedMap.put(3.40, passingFlywheelSpeed1.get());
    passingFlywheelSpeedMap.put(4.275, passingFlywheelSpeed2.get());
    passingFlywheelSpeedMap.put(5.46, passingFlywheelSpeed3.get());
    passingFlywheelSpeedMap.put(6.62, passingFlywheelSpeed4.get());
    passingFlywheelSpeedMap.put(7.80, passingFlywheelSpeed5.get());
    passingFlywheelSpeedMap.put(8.67, passingFlywheelSpeed6.get());
    passingFlywheelSpeedMap.put(9.65, passingFlywheelSpeed7.get());
    passingFlywheelSpeedMap.put(10.16, passingFlywheelSpeed8.get());
    passingFlywheelSpeedMap.put(17.16, passingFlywheelSpeedLast.get());
  }

  private static void loadPassingTimeOfFlightMap() {
    passingTimeOfFlightMap.put(3.40, 1.10);
    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.16);
  }
}