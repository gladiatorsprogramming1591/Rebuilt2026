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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Bounds;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShooterCalculation {
  private static ShooterCalculation instance;

  @Getter private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
private static final LoggedTunableNumber velocityCompensationScalar =
    new LoggedTunableNumber("ShooterCalculation/VelocityCompensationScalar", 1.0);
  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  private static String updateHoodMapKey = "ShooterCalculation/UpdateMaps/HoodAngleMap/Update Hood Angles";
  private static String updateFlywheelMapKey = "ShooterCalculation/UpdateMaps/FlyWheelSpeedMap/Update Flywheel Speeds";
  private static String updatePassingHoodMapKey = "ShooterCalculation/UpdateMaps/PassingHoodMap/Update Passing Hood Angles";
  private static String updatePassingFlywheelMapKey = "ShooterCalculation/UpdateMaps/PassingFlywheelMap/Update Passing Flywheel Speeds";

  public static ShooterCalculation getInstance() {
    if (instance == null) instance = new ShooterCalculation();
    return instance;
  }

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

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Double> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
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
          new LoggedTunableNumber(
              "ShooterCalculation/Presets/HoodMin/HoodAngle", Units.radiansToDegrees(0)),
          new LoggedTunableNumber("ShooterCalculation/Presets/HoodMin/FlywheelSpeed", 2000));
  public static final LaunchPreset hoodMaxPreset =
      new LaunchPreset(
          new LoggedTunableNumber(
              "ShooterCalculation/Presets/HoodMax/HoodAngle", Units.radiansToDegrees(700)),
          new LoggedTunableNumber("ShooterCalculation/Presets/HoodMax/FlywheelSpeed", 2000));

  public static final LoggedTunableNumber passingIdleSpeed =
      new LoggedTunableNumber("ShooterCalculation/PassingIdleSpeed", 2000);

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // Passing targets
  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
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
      new LoggedTunableNumber("HoodAngleMap/0.0", 50.0);
  private static final LoggedTunableNumber hoodAngle2 =
      new LoggedTunableNumber("HoodAngleMap/1.46", 70.0); // 50
  private static final LoggedTunableNumber hoodAngle3 =
      new LoggedTunableNumber("HoodAngleMap/1.73", 70.0); // 60
  private static final LoggedTunableNumber hoodAngle4 =
      new LoggedTunableNumber("HoodAngleMap/2.18", 100.0);
  private static final LoggedTunableNumber hoodAngle5 =
      new LoggedTunableNumber("HoodAngleMap/2.47", 120.0);
  private static final LoggedTunableNumber hoodAngle6 =
      new LoggedTunableNumber("HoodAngleMap/2.70", 315.0); // 0.0
  private static final LoggedTunableNumber hoodAngle7 =
      new LoggedTunableNumber("HoodAngleMap/2.94", 330.0);
  private static final LoggedTunableNumber hoodAngle8 =
      new LoggedTunableNumber("HoodAngleMap/3.48", 450.0);
  private static final LoggedTunableNumber hoodAngle9 =
      new LoggedTunableNumber("HoodAngleMap/3.92", 550.0);
  private static final LoggedTunableNumber hoodAngle10 =
      new LoggedTunableNumber("HoodAngleMap/4.35", 660.0);
  private static final LoggedTunableNumber hoodAngle11 =
      new LoggedTunableNumber("HoodAngleMap/4.84", 750.0); 
  private static final LoggedTunableNumber hoodAngle12 =
      new LoggedTunableNumber("HoodAngleMap/5.46", 800.0); // we noticed that 750 was reaching the max for the hood, after is a deadspot.

  private static final LoggedTunableNumber flywheelSpeed1 =
      new LoggedTunableNumber("FlyWheelSpeedMap/0.0", 1450.0);
  private static final LoggedTunableNumber flywheelSpeed2 =
      new LoggedTunableNumber("FlyWheelSpeedMap/0.96", 1450.0);
  private static final LoggedTunableNumber flywheelSpeed3 =
      new LoggedTunableNumber("FlyWheelSpeedMap/1.46", 1650.0); // 1600
  private static final LoggedTunableNumber flywheelSpeed4 =
      new LoggedTunableNumber("FlyWheelSpeedMap/1.73", 1750.0); // 1650
  private static final LoggedTunableNumber flywheelSpeed5 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.18", 1900.0);
  private static final LoggedTunableNumber flywheelSpeed6 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.47", 1950.0);
  private static final LoggedTunableNumber flywheelSpeed7 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.70", 1950.0);
  private static final LoggedTunableNumber flywheelSpeed8 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.94", 1950.0);
  private static final LoggedTunableNumber flywheelSpeed9 =
      new LoggedTunableNumber("FlyWheelSpeedMap/3.48", 2024.0);
  private static final LoggedTunableNumber flywheelSpeed10 =
      new LoggedTunableNumber("FlyWheelSpeedMap/3.92", 2050.0);
  private static final LoggedTunableNumber flywheelSpeed11 =
      new LoggedTunableNumber("FlyWheelSpeedMap/4.35", 2125.0);
  private static final LoggedTunableNumber flywheelSpeed12 =
      new LoggedTunableNumber("FlyWheelSpeedMap/4.84", 2200.0);
  private static final LoggedTunableNumber flywheelSpeed13 =
      new LoggedTunableNumber("FlyWheelSpeedMap/5.46", 2350.0);

  private static final LoggedTunableNumber passingHoodAngle1 =
      new LoggedTunableNumber("PassingHoodMap/3.4", 700.0);
  private static final LoggedTunableNumber passingHoodAngle2 =
      new LoggedTunableNumber("PassingHoodMap/4.275", 700.0); // Bumper against bump 
  private static final LoggedTunableNumber passingHoodAngle3 =
      new LoggedTunableNumber("PassingHoodMap/5.46", 800.0); // 240
  private static final LoggedTunableNumber passingHoodAngle4 =
      new LoggedTunableNumber("PassingHoodMap/6.62", 800.0); // 400
  private static final LoggedTunableNumber passingHoodAngle5 =
      new LoggedTunableNumber("PassingHoodMap/7.8", 800.0); // 500
  private static final LoggedTunableNumber passingHoodAngle6 =
      new LoggedTunableNumber("PassingHoodMap/8.67", 800.0); // 3/4 into NZ
  private static final LoggedTunableNumber passingHoodAngle7 =
      new LoggedTunableNumber("PassingHoodMap/9.65", 800.0); // NZ opposing bump
  private static final LoggedTunableNumber passingHoodAngle8 =
      new LoggedTunableNumber("PassingHoodMap/10.16", 800.0); // NZ opposing trench
// Within opposing AZ
//   private static final LoggedTunableNumber passingHoodAngle9 =
//       new LoggedTunableNumber("PassingHoodMap/11.68", 2250.0); // Opposing AZ bump
//   private static final LoggedTunableNumber passingHoodAngle10 =
//       new LoggedTunableNumber("PassingHoodMap/13.29", 2250.0); // Opposing AZ 1/2
//   private static final LoggedTunableNumber passingHoodAngle11 =
//       new LoggedTunableNumber("PassingHoodMap/14.81", 2250.0); // Opposing AZ wall
  private static final LoggedTunableNumber passingHoodAngleLast =
      new LoggedTunableNumber("PassingHoodMap/17.16", 800.0); // 500 1009 is approx max

  private static final LoggedTunableNumber passingFlywheelSpeed1 =
      new LoggedTunableNumber("PassingFlywheelMap/3.4", 1700.0); // 800
  private static final LoggedTunableNumber passingFlywheelSpeed2 =
      new LoggedTunableNumber("PassingFlywheelMap/4.275", 1700.0); // Bumper against bump
  private static final LoggedTunableNumber passingFlywheelSpeed3 =
      new LoggedTunableNumber("PassingFlywheelMap/5.46", 2000.0); // 1008
  private static final LoggedTunableNumber passingFlywheelSpeed4 =
      new LoggedTunableNumber("PassingFlywheelMap/6.62", 2150.0); // 1134
  private static final LoggedTunableNumber passingFlywheelSpeed5 =
      new LoggedTunableNumber("PassingFlywheelMap/7.8", 2250.0); // 1260
  private static final LoggedTunableNumber passingFlywheelSpeed6 =
      new LoggedTunableNumber("PassingFlywheelMap/8.67", 2325.0); // 3/4 into NZ
  private static final LoggedTunableNumber passingFlywheelSpeed7 =
      new LoggedTunableNumber("PassingFlywheelMap/9.65", 2425.0); // NZ opposing bump
  private static final LoggedTunableNumber passingFlywheelSpeed8 =
      new LoggedTunableNumber("PassingFlywheelMap/10.16", 2475.0); // NZ opposing trench
// Within opposing AZ
//   private static final LoggedTunableNumber passingFlywheelSpeed6 =
//       new LoggedTunableNumber("PassingFlywheelMap/11.68", 2250.0); // Opposing AZ bump
//   private static final LoggedTunableNumber passingFlywheelSpeed6 =
//       new LoggedTunableNumber("PassingFlywheelMap/13.29", 2250.0); // Opposing AZ 1/2
//   private static final LoggedTunableNumber passingFlywheelSpeed6 =
//       new LoggedTunableNumber("PassingFlywheelMap/14.81", 2250.0); // Opposing AZ wall
  private static final LoggedTunableNumber passingFlywheelSpeedLast =
      new LoggedTunableNumber("PassingFlywheelMap/17.16", 4000.0); // 2300


  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 5.4;
    passingMaxDistance = 17.16;
    phaseDelay = 0.03;

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

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(3.40, passingHoodAngle1.get());
    passingHoodAngleMap.put(4.275, passingHoodAngle2.get());
    passingHoodAngleMap.put(5.46, passingHoodAngle3.get());
    passingHoodAngleMap.put(6.62, passingHoodAngle4.get());
    passingHoodAngleMap.put(7.80, passingHoodAngle5.get());
    passingHoodAngleMap.put(8.67, passingHoodAngle6.get());
    passingHoodAngleMap.put(9.65, passingHoodAngle7.get());
    passingHoodAngleMap.put(10.16, passingHoodAngle8.get());
    passingHoodAngleMap.put(17.16, passingHoodAngleLast.get());

    passingFlywheelSpeedMap.put(3.40, passingFlywheelSpeed1.get());
    passingFlywheelSpeedMap.put(4.275, passingFlywheelSpeed2.get());
    passingFlywheelSpeedMap.put(5.46, passingFlywheelSpeed3.get());
    passingFlywheelSpeedMap.put(6.62, passingFlywheelSpeed4.get());
    passingFlywheelSpeedMap.put(7.80, passingFlywheelSpeed5.get());
    passingFlywheelSpeedMap.put(8.67, passingFlywheelSpeed6.get());
    passingFlywheelSpeedMap.put(9.65, passingFlywheelSpeed7.get());
    passingFlywheelSpeedMap.put(10.16, passingFlywheelSpeed8.get());
    passingFlywheelSpeedMap.put(17.16, passingFlywheelSpeedLast.get());

    passingTimeOfFlightMap.put(3.40, 1.10);
    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.16);

    passingPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Passing/HoodAngle",
                hoodAngleMap.get(passingPresetDistance)),
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Passing/FlywheelSpeed",
                flywheelSpeedMap.get(passingPresetDistance)));
    hubPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Hub/HoodAngle", hoodAngleMap.get(hubPresetDistance)),
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Hub/FlywheelSpeed",
                flywheelSpeedMap.get(hubPresetDistance)));
    towerPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Tower/HoodAngle",
                hoodAngleMap.get(towerPresetDistance)),
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Tower/FlywheelSpeed",
                flywheelSpeedMap.get(towerPresetDistance)));
    trenchPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Trench/HoodAngle",
                hoodAngleMap.get(trenchPresetDistance)),
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Trench/FlywheelSpeed",
                flywheelSpeedMap.get(trenchPresetDistance)));
    outpostPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Outpost/HoodAngle",
                hoodAngleMap.get(outpostPresetDistance)),
            new LoggedTunableNumber(
                "ShooterCalculation/Presets/Outpost/FlywheelSpeed",
                flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public void updateMaps() {
    if (SmartDashboard.getBoolean(updateHoodMapKey, true)) {
        SmartDashboard.putBoolean(updateHoodMapKey, false);
        hoodAngleMap.put(0.0, hoodAngle1.get());
        hoodAngleMap.put(1.46, hoodAngle2.get()); // 50
        hoodAngleMap.put(1.73, hoodAngle3.get()); // 60
        hoodAngleMap.put(2.18, hoodAngle4.get());
        hoodAngleMap.put(2.47, hoodAngle5.get());
        hoodAngleMap.put(2.70, hoodAngle6.get()); // 0.0
        hoodAngleMap.put(2.94, hoodAngle7.get());
        hoodAngleMap.put(3.48, hoodAngle8.get());
        hoodAngleMap.put(3.92, hoodAngle9.get());
        hoodAngleMap.put(4.35, hoodAngle10.get());
        hoodAngleMap.put(4.84, hoodAngle11.get());
        hoodAngleMap.put(5.46, hoodAngle12.get());
    }

    if (SmartDashboard.getBoolean(updateFlywheelMapKey, true)) {
        SmartDashboard.putBoolean(updateFlywheelMapKey, false);
        flywheelSpeedMap.clear();
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

    if (SmartDashboard.getBoolean(updatePassingHoodMapKey, true)) {
        SmartDashboard.putBoolean(updatePassingHoodMapKey, false);
        passingHoodAngleMap.put(3.40, passingHoodAngle1.get());
        passingHoodAngleMap.put(4.275, passingHoodAngle2.get()); 
        passingHoodAngleMap.put(5.46, passingHoodAngle3.get()); 
        passingHoodAngleMap.put(6.62, passingHoodAngle4.get()); 
        passingHoodAngleMap.put(7.80, passingHoodAngle5.get());
        passingHoodAngleMap.put(8.67, passingHoodAngle6.get());
        passingHoodAngleMap.put(9.65, passingHoodAngle7.get());
        passingHoodAngleMap.put(10.16, passingHoodAngle8.get()); 
        passingHoodAngleMap.put(17.16, passingHoodAngleLast.get()); 
    }

    if (SmartDashboard.getBoolean(updatePassingFlywheelMapKey, true)) {
        SmartDashboard.putBoolean(updatePassingFlywheelMapKey, false);
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
  }

  public LaunchingParameters getParameters() {
    if (Constants.tuningMode) {
        updateMaps();
    }

    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getRobotPoseField().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getRobotPoseField();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getMeasuredRobotRelativeSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        passing
            ? getPassingTarget()
            : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher.toTransform2d());
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity = RobotState.getInstance().getMeasuredFieldRelativeSpeeds();
    var robotAngle = RobotState.getInstance().getYawForVision();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : GeomUtil.transformVelocity(
                robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    boolean doLookAhead = true;
    if (doLookAhead) {
      for (int i = 0; i < 20; i++) {
        timeOfFlight =
            passing
                ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
                : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
        double offsetX =
    launcherVelocity.vxMetersPerSecond
        * timeOfFlight
        * velocityCompensationScalar.get();

double offsetY =
    launcherVelocity.vyMetersPerSecond
        * timeOfFlight
        * velocityCompensationScalar.get();
        lookaheadPose =
            new Pose2d(
                launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                launcherPosition.getRotation());
        lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
      }
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(robotToLauncher.toTransform2d().inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance)
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance);
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    double flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle + Units.degreesToRotations(hoodAngleOffsetDeg),
            hoodVelocity,
            flywheelVelocity,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput(
        "ShooterCalculation/HoodAngle", hoodAngle + Units.degreesToRotations(hoodAngleOffsetDeg));
    Logger.recordOutput("ShooterCalculation/Passing", passing);
    Logger.recordOutput("ShooterCalculation/Target", target);
    Logger.recordOutput("ShooterCalculation/HoodVelocity", hoodVelocity);
    Logger.recordOutput("ShooterCalculation/FlywheelVelocity", flywheelVelocity);
    Logger.recordOutput("ShooterCalculation/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("ShooterCalculation/LookaheadPose", lookaheadRobotPose);
    Logger.recordOutput(
        "ShooterCalculation/LookaheadLauncherToTargetDistance", lookaheadLauncherToTargetDistance);
    Logger.recordOutput("ShooterCalculation/LauncherToTargetDistance", launcherToTargetDistance);

    return latestParameters;
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
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(RobotState.getInstance().getRobotPoseField()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, flippedY));

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }
}
