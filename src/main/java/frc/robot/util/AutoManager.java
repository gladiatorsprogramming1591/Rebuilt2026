package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoManager {
  private static final String AUTO_FILE_EXTENSION = ".auto";

  private final SendableChooser<Command> autos = new SendableChooser<>();
  private final Drive drivetrain;

  public AutoManager(Drive drivetrain) {
    this.drivetrain = drivetrain;

    autos.setDefaultOption("Do Nothing", Commands.none());
    preloadPathPlannerAutos();
  }

  private void preloadPathPlannerAutos() {
    int loadedAutoCount = 0;
    for (String autoName : getDeployedAutoNames()) {
      try {
        PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        autos.addOption(autoName, wrapAuto(new PathPlannerAuto(autoName)));
        autos.addOption("Left " + autoName, wrapAuto(new PathPlannerAuto(autoName, true)));
        loadedAutoCount++;
      } catch (Exception exception) {
        reportSkippedAuto(autoName, exception);
      }
    }

    Logger.recordOutput("AutoManager/LoadedAutoCount", loadedAutoCount);
  }

  public void warmupAutos() {
  for (String autoName : AutoBuilder.getAllAutoNames()) {
    try {
      PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      new PathPlannerAuto(autoName);
      new PathPlannerAuto(autoName, true);
    } catch (Exception exception) {
      DriverStation.reportWarning(
          "Failed to warm up PathPlanner auto: " + autoName + " - " + exception.getMessage(),
          exception.getStackTrace());
    }
  }
}

  private List<String> getDeployedAutoNames() {
    Path autosDirectory =
        Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
    List<String> autoNames = new ArrayList<>();

    if (!Files.isDirectory(autosDirectory)) {
      DriverStation.reportWarning("PathPlanner autos directory was not found: " + autosDirectory, false);
      return autoNames;
    }

    try (Stream<Path> autoFiles = Files.list(autosDirectory)) {
      autoFiles
          .filter(Files::isRegularFile)
          .map((path) -> path.getFileName().toString())
          .filter((fileName) -> fileName.endsWith(AUTO_FILE_EXTENSION))
          .map(this::stripAutoExtension)
          .sorted()
          .forEach(autoNames::add);
    } catch (IOException exception) {
      DriverStation.reportWarning("Failed to list PathPlanner autos: " + exception.getMessage(), false);
    }

    return autoNames;
  }

  private Command wrapAuto(Command autoCommand) {
    return Commands.runOnce(drivetrain::stop, drivetrain).andThen(autoCommand);
  }

  private String stripAutoExtension(String fileName) {
    return fileName.substring(0, fileName.length() - AUTO_FILE_EXTENSION.length());
  }

  private void reportSkippedAuto(String autoName, Exception exception) {
    String message = "Skipping PathPlanner auto '" + autoName + "': " + exception.getMessage();
    DriverStation.reportWarning(message, false);
    Logger.recordOutput("AutoManager/SkippedAutos/" + sanitizeKey(autoName), message);
  }

  private String sanitizeKey(String value) {
    return value.replaceAll("[^A-Za-z0-9_]", "_");
  }

  public SendableChooser<Command> getChooser() {
    return autos;
  }
}
