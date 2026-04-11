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

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

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
      new LoggedTunableNumber("HoodAngleMap/0.0", 50);
  private static final LoggedTunableNumber hoodAngle2 =
      new LoggedTunableNumber("HoodAngleMap/1.46", 70);
  private static final LoggedTunableNumber hoodAngle3 =
      new LoggedTunableNumber("HoodAngleMap/1.73", 70);
  private static final LoggedTunableNumber hoodAngle4 =
      new LoggedTunableNumber("HoodAngleMap/2.18", 90);
  private static final LoggedTunableNumber hoodAngle5 =
      new LoggedTunableNumber("HoodAngleMap/2.47", 120);
  private static final LoggedTunableNumber hoodAngle6 =
      new LoggedTunableNumber("HoodAngleMap/2.70", 150);
  private static final LoggedTunableNumber hoodAngle7 =
      new LoggedTunableNumber("HoodAngleMap/2.94", 180);
  private static final LoggedTunableNumber hoodAngle8 =
      new LoggedTunableNumber("HoodAngleMap/3.48", 210);
  private static final LoggedTunableNumber hoodAngle9 =
      new LoggedTunableNumber("HoodAngleMap/3.92", 240);
  private static final LoggedTunableNumber hoodAngle10 =
      new LoggedTunableNumber("HoodAngleMap/4.35", 350);
  private static final LoggedTunableNumber hoodAngle11 =
      new LoggedTunableNumber("HoodAngleMap/4.84", 400);
  private static final LoggedTunableNumber hoodAngle12 =
      new LoggedTunableNumber("HoodAngleMap/5.46", 500);

  private static final LoggedTunableNumber flywheelSpeed1 =
      new LoggedTunableNumber("FlyWheelSpeedMap/0.0", 1500);
  private static final LoggedTunableNumber flywheelSpeed2 =
      new LoggedTunableNumber("FlyWheelSpeedMap/0.96", 1500);
  private static final LoggedTunableNumber flywheelSpeed3 =
      new LoggedTunableNumber("FlyWheelSpeedMap/1.46", 1600);
  private static final LoggedTunableNumber flywheelSpeed4 =
      new LoggedTunableNumber("FlyWheelSpeedMap/1.73", 1650);
  private static final LoggedTunableNumber flywheelSpeed5 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.18", 1700);
  private static final LoggedTunableNumber flywheelSpeed6 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.47", 1750);
  private static final LoggedTunableNumber flywheelSpeed7 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.70", 1800);
  private static final LoggedTunableNumber flywheelSpeed8 =
      new LoggedTunableNumber("FlyWheelSpeedMap/2.94", 1850);
  private static final LoggedTunableNumber flywheelSpeed9 =
      new LoggedTunableNumber("FlyWheelSpeedMap/3.48", 1900);
  private static final LoggedTunableNumber flywheelSpeed10 =
      new LoggedTunableNumber("FlyWheelSpeedMap/3.92", 200);
  private static final LoggedTunableNumber flywheelSpeed11 =
      new LoggedTunableNumber("FlyWheelSpeedMap/4.35", 2000);
  private static final LoggedTunableNumber flywheelSpeed12 =
      new LoggedTunableNumber("FlyWheelSpeedMap/4.84", 2000);
  private static final LoggedTunableNumber flywheelSpeed13 =
      new LoggedTunableNumber("FlyWheelSpeedMap/5.46", 2000);

  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 5.4;
    passingMaxDistance = 17.16;
    phaseDelay = 0.03;

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

    flywheelSpeedMap.put(0.0, flywheelSpeed1.get());
    flywheelSpeedMap.put(0.96, flywheelSpeed2.get());
    flywheelSpeedMap.put(1.46, flywheelSpeed3.get()); // 1500
    flywheelSpeedMap.put(1.73, flywheelSpeed4.get()); // 1650
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

    passingHoodAngleMap.put(3.40, 700.0);
    passingHoodAngleMap.put(4.275, 700.0); // Bumper against bump 
    passingHoodAngleMap.put(5.46, 800.0); // 240
    passingHoodAngleMap.put(6.62, 950.0); // 400
    passingHoodAngleMap.put(7.80, 1000.0); // 500
    passingHoodAngleMap.put(17.16, 1009.0); // 500 1009 is approx max

    passingFlywheelSpeedMap.put(3.40, 1700.0); // 800
    passingFlywheelSpeedMap.put(4.275, 1700.0); // Bumper against bump
    passingFlywheelSpeedMap.put(5.46, 2000.0); // 1008
    passingFlywheelSpeedMap.put(6.62, 2500.0); // 1134
    passingFlywheelSpeedMap.put(7.80, 3800.0); // 1260
    passingFlywheelSpeedMap.put(17.16, 6000.0); // 2300

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
    if (SmartDashboard.getBoolean("HoodAngleMap/Update Hood Angles", true)) {
        SmartDashboard.getBoolean("HoodAngleMap/Update Hood Angles", false);
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

    if (SmartDashboard.getBoolean("HoodAngleMap/Update Flywheel Speeds", true)) {
        SmartDashboard.putBoolean("HoodAngleMap/Update Flywheel Speeds", false);
        flywheelSpeedMap.clear();
        flywheelSpeedMap.put(0.0, flywheelSpeed1.get());
        flywheelSpeedMap.put(0.96, flywheelSpeed2.get());
        flywheelSpeedMap.put(1.46, flywheelSpeed3.get()); // 1500
        flywheelSpeedMap.put(1.73, flywheelSpeed4.get()); // 1650
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
        double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
        double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
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
