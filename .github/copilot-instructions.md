## Purpose

Short, actionable guidance for an AI code assistant working in this repository (Rebuilt2026). Focus on patterns, build & runtime workflows, and concrete file references so you can be immediately productive.

## Big-picture architecture (what to know first)
- Command-based WPILib robot using AdvantageKit logging. Entry points: `frc.robot.Robot`, `frc.robot.RobotContainer`, and `frc.robot.Main`.
- Subsystems follow an "IO interface + implementations" pattern. Each subsystem (example: `src/main/java/frc/robot/subsystems/shooter`) provides:
  - `ShooterIO` (interface with `updateInputs(...)` and `runShooter(...)`)
  - hardware implementation: `ShooterIOKraken` (real robot / CTRE Phoenix 6)
  - sim implementation: `ShooterIOSim` (physics sim)
  - subsystem wrapper: `Shooter` (calls `io.updateInputs` in `periodic()` and uses `io.runShooter(...)`)
- `RobotContainer` wires implementations based on `Constants.currentMode` (REAL / SIM / REPLAY). Follow this pattern when adding new hardware.

## Key files and directories (quick map)
- `src/main/java/frc/robot/Robot.java` — logger setup and mode (REAL/SIM/REPLAY) handling.
- `src/main/java/frc/robot/RobotContainer.java` — constructs subsystems and binds buttons/commands.
- `src/main/java/frc/robot/subsystems/*` — subsystem packages; look for `*IO`, `*IOSim`, `*IOKraken` naming.
- `src/main/java/frc/robot/util/LoggedTunableNumber.java` — project tunable number pattern (dashboard integration is currently stubbed).
- `build.gradle` — GradleRIO setup, tasks, and formatting (spotless) bindings.

## Conventions & patterns to follow when editing
- Dependency injection for hardware: always add a new `XxxIO` interface and at least a sim and real implementation. Inject from `RobotContainer` via `Constants.currentMode`.
- Keep `updateInputs(...)` and `run...(...)` methods on IO types thin: they should be the only classes that touch hardware or simulation primitives.
- Tuning and dashboard values use `LoggedTunableNumber` (see `Shooter.java` for examples). These are declared as static fields and used inside commands or periodic checks.
- Logging via AdvantageKit (`org.littletonrobotics.junction.Logger`) is global — use `Logger.processInputs("Subsystem", inputs)` in `periodic()` snapshots.

## Build, test, and runtime workflows (concrete commands)
- Build locally (Windows PowerShell):
  - .\gradlew.bat build
  - .\gradlew.bat test
- Deploy to RoboRIO: .\gradlew.bat deploy (GradleRIO will read the team number from WPILib prefs or you can pass `-Pteam=#### `).
- Run replay/watch helper (project defines a `replayWatch` task): .\gradlew.bat replayWatch
- Note: spotless formatting runs as part of compilation (`spotlessApply` is a compile dependency). If you see formatting failures, run: .\gradlew.bat spotlessApply

## Runtime modes and logs
- Modes are controlled by `Constants.simMode` and `Constants.currentMode`. Change `simMode` to toggle default simulation vs replay behavior when not on a RoboRIO.
- Logger setup lives in `Robot.java` — for REPLAY the code calls `LogFileUtil.findReplayLog()` and `Logger.setReplaySource(...)`. To replay logs, set `simMode` to `REPLAY` or use the project's log-replay tooling.

## Integration points & external deps (what to watch for)
- CTRE Phoenix 6 (Kraken) — hardware controllers: many `*IOKraken` classes use `com.ctre.phoenix6` APIs.
- PathPlanner (`com.pathplanner.lib`) — auto builder and chooser setup in `RobotContainer`.
- PhotonVision simulation — `subsystems/vision/CameraIOSim` uses Photon sim APIs.
- AdvantageKit (org.littletonrobotics.junction) — logging, replay, and networktable publishing.
- Generated artifacts: `TunerConstants` and `BuildConstants` are generated at build time (see `gversion` plugin configuration in `build.gradle`). Don't hand-edit generated files.

## Small examples to follow
- Adding new subsystem: mirror `subsystems/shooter` layout — `XxxIO`, `XxxIOKraken`, `XxxIOSim`, `Xxx.java` subsystem wrapper, and wire in `RobotContainer` with the same branching on `Constants.currentMode`.
- Read inputs and log them in `periodic()` using the `Logger.processInputs("Name", inputs)` pattern.

## What to avoid / gotchas
- Don't call hardware APIs from outside the `*IO` implementations — keep hardware access centralized.
- `LoggedTunableNumber.get()` is currently stubbed to return 0.0 in this template; double-check tuning flows before relying on dashboard values.
- Formatting is enforced via Spotless and runs on compile; fix formatting locally with `spotlessApply` to avoid CI breaks.

## If you need more context
- Inspect `RobotContainer.java` for the wiring pattern and `subsystems/*` for repeated examples. If you want a walkthrough for a particular subsystem (e.g., drive or vision), tell me which one and I'll summarize the I/O, sim, and commands.

---
If anything here looks off or you'd like the file to include additional examples (debugging tips, CI commands, or a brief code tour of `drive`), tell me which area to expand and I will update this doc.
