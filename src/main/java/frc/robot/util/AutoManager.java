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
  private String rightSideRushName = "Right Rush to NZ";
  private String leftSideRushHub = "Bottom Rush to Hub";
  private String middleShootPreload = "Middle Shoot Preload";

  private PathConstraints constraints = new PathConstraints(7, 3, 1, 1);

  public AutoManager(Drive drivetrain) {
    this.drivetrain = drivetrain;

    autos = new SendableChooser<>();
    autos.addOption("Right Side Rush", wrapAutoWithPose(new PathPlannerAuto(rightSideRushName)));
    autos.addOption(
        "Left Side Rush", wrapAutoWithPose(new PathPlannerAuto(rightSideRushName, true)));
    autos.addOption(
        "Left Side Rush HUB", wrapAutoWithPose(new PathPlannerAuto(leftSideRushHub, true)));
    autos.addOption(
      "Middle Shoot Preload", wrapAutoWithPose(new PathPlannerAuto(middleShootPreload)));
  }

  private Command wrapAutoWithPose(PathPlannerAuto autoCommand) {
    return new InstantCommand(
        () -> {
          new PathPlannerAuto(autoCommand, drivetrain.getPose()).schedule();
        });
  }

  public SendableChooser<Command> getChooser() {
    return autos;
  }
}
