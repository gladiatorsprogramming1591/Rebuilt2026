package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoManager {
  private SendableChooser<Command> autos;
  private Drive drivetrain;
  // FYI: Auto names are case-sensitive!
  private String rightSideRushHub = "Bottom Rush to Hub";
  private String middleShootStraight = "Middle Shoot Straight";
  private String middleShootTop = "Middle Shoot Top";
  private String middleShootBottom = "Middle Shoot Bottom";
  private String rightSideBumpReturn = "Bottom bump";
  private String rightSideBumpWiggle = "Bottom Bump wiggle";
  private String rightSideWiggles = "MORE WIGGLES";
  private String rightSideWigglesTrench = "MORE WIGGLES but trench";
  private String rightSideBumps = "MORE BUMPS";
  private String middleShoot = "Middle shoot";
  private String middleOutpost = "Middle outpost";
  private String rightSideWigglesTrenchWithDelay = "MORE WIGGLES but trench with delay";
  private String rightSafeBeanTag = "SAFE BEAN Tag";

  private PathConstraints constraints = new PathConstraints(7, 3, 1, 1);

  public AutoManager(Drive drivetrain) {
    this.drivetrain = drivetrain;

    autos = new SendableChooser<>();
    // autos.addOption("Right Side Rush HUB", wrapAutoWithPose(new PathPlannerAuto(rightSideRushHub)));
    // autos.addOption(
    //     "Left Side Rush HUB", wrapAutoWithPose(new PathPlannerAuto(rightSideRushHub, true)));
    // autos.addOption(
    //     "Middle Shoot Straight", wrapAutoWithPose(new PathPlannerAuto(middleShootStraight)));
    // autos.addOption("Middle Shoot Top", wrapAutoWithPose(new PathPlannerAuto(middleShootTop)));
    // autos.addOption(
    //     "Middle Shoot Bottom", wrapAutoWithPose(new PathPlannerAuto(middleShootBottom)));
    autos.addOption("Left Side Wiggle", wrapAutoWithPose(new PathPlannerAuto(rightSideWiggles, true)));
    autos.addOption(
        "Right Side Wiggle", wrapAutoWithPose(new PathPlannerAuto(rightSideWiggles)));
    autos.addOption("Left Side Wiggles trench", wrapAutoWithPose(new PathPlannerAuto(rightSideWigglesTrench, true)));
    autos.addOption(
      "Right Side Wiggles trench", wrapAutoWithPose(new PathPlannerAuto(rightSideWigglesTrench)));
      autos.addOption(
        "Delayed Left Side Wiggles trench", wrapAutoWithPose(new PathPlannerAuto(rightSideWigglesTrenchWithDelay, true)));
    autos.addOption(
      "Delayed Right Side Wiggles trench", wrapAutoWithPose(new PathPlannerAuto(rightSideWigglesTrenchWithDelay)));
    autos.addOption("Center Back up and shoot", wrapAutoWithPose(new PathPlannerAuto(middleShoot)));
    autos.addOption("Center Outpost", wrapAutoWithPose(new PathPlannerAuto(middleOutpost)));
    autos.addOption("Right SAFE BEAN Tag", wrapAutoWithPose(new PathPlannerAuto(rightSafeBeanTag)));
    autos.addOption("Left SAFE BEAN Tag", wrapAutoWithPose(new PathPlannerAuto(rightSafeBeanTag, true)));
  }

  private Command wrapAutoWithPose(PathPlannerAuto autoCommand) {
    return new InstantCommand(
        () -> {
          // TODO: .schedule is deprecated, and starting pose is not seeded. Consider removal.
          // Passing in pose is only used for the getStartingPose method; does not actually seed
          // auto's
          // starting pose.
          new PathPlannerAuto(autoCommand, drivetrain.getPose()).schedule();
        });
  }

  public SendableChooser<Command> getChooser() {
    return autos;
  }
}
