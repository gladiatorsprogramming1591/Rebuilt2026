package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.*;

public class AutoManager {
  private SendableChooser<Command> autos;
  private Drive drivetrain;
  private String rightSideRushHub = "Bottom Rush to Hub";
  private String middleShootStraight = "Middle Shoot Straight";
  private String middleShootTop = "Middle Shoot Top";
  private String middleShootBottom = "Middle Shoot Bottom";

  private PathConstraints constraints = new PathConstraints(7, 3, 1, 1);

  public AutoManager(Drive drivetrain) {
    this.drivetrain = drivetrain;

    autos = new SendableChooser<>();
    autos.addOption("Right Side Rush HUB", wrapAutoWithPose(new PathPlannerAuto(rightSideRushHub)));
    autos.addOption(
        "Left Side Rush HUB", wrapAutoWithPose(new PathPlannerAuto(rightSideRushHub, true)));
    autos.addOption(
        "Middle Shoot Straight", wrapAutoWithPose(new PathPlannerAuto(middleShootStraight)));
    autos.addOption("Middle Shoot Top", wrapAutoWithPose(new PathPlannerAuto(middleShootTop)));
    autos.addOption(
        "Middle Shoot Bottom", wrapAutoWithPose(new PathPlannerAuto(middleShootBottom)));
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
