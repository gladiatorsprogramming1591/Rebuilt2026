High Priority:
--------------
- Investigate shoot from NZ passing map (can test via sim)
#### Code Refactor:
- Ensure inputs/outputs are being used/updated/logged properly across subsystems' IO, Real, and Sim
    - Consider using outputs when applying control requests to motors (where not already used)
- Command structure & organization:
    - Commands within RobotContainer should really be in their own subsystem, or in a new class under the "commands" folder if utilizing multiple subsystems (e.g. create shootCommands class to contain all 4 of our shoot commands)
- Mark unused/outdated commands or methods for removal within the subsystems & robotContainer
- find and resolve inconsistencies across subsystems & robotContainer
- Consider integrating toward state-based
#### Battery Draw Management
- Turn off all idling motors (shooter, intake, hood, etc.) if a brownout is detected near the end of a match (where we're unlikely to shoot)
- investigate loss of power of intake when attempting fuel pick up before at speed (Potential fix: run intake automatically when hopper extended [if state-based; while not in intaking state])

Mid Priority:
-------------
* Boolean supplier for when intake is fully extended and fully stowed (monitor for a spike in current over time when deploy motor stalls against a hardstop [or fuel w/ lower current spike])
* Stow automatically
* Verify that each subsystem is logging important motor info (current, voltage, velocity, position, etc.) via status codes
* Review TODO in shooter periodic: shooter periodic sets shooter to a duty cycle of 0 when not in DUTYCYCLE mode

Low Priority:
-------------

- "driveCurrentLimit": 15.0 in pathplanner?
- Verify whether PP_CONFIG for PathPlanner overrides our app settings. Consider tuning/matching configs with app
- Investigate why Operator intake only buttons (POV up/down) were not working
- Fine-tuning:

    - Tune kSpeedAt12Volts to be true max theoretical free speed (m/s) at 12 V applied output.
        - Apply 12V to each drive motor using phoenix tuner, get motor velocities (rot/sec), get average
        - kSpeedAt12Volts = average motor vel (rot/sec) / gearing (kDriveGearRatio) * wheel circumference (w/ new tread in meters)
    - Review driveCommands PIDs (effects teleop drive)
    - Make Shooter and hood slot0 configs (e.g. outputs.kP) to be LoggedTunableNumbers.
        - Apply configs only when needed (e.g. a SmartDashboard button)

- High Difficulty:
    - Code deploy hanging at 86% issue
        - Same issue between different rio2's and computers
    - Logger stopping after a short duration
    - Stop spotlessGradle from auto-formatting on build
- Sim:
    - Implement more sim methods
    - Sim using TalonFXS


MISC
----
- Systems check list
- Go through all TODOs in code (over 25)
- Ideal camera placement/mount
- Is there a benefit to being able to interrupt the intake without interrupting the deploy? Can this be done without separating the subsystem?







<br><br><br><br><br><br><br><br>







<br><br><br><br><br><br><br><br>
Completed
---------
* Update vendor dependencies
* *- change path to be even closer to hub on second rush to center, and stay in NZ
- is drivetrain maxed out? A: No, kSpeedAt12Volts from 10 to 15 increased drive speed. reducing to 13.
* *ALLIANCE REQ**: Move shoot pos closer to hub
* NEEDS TO TEST
    - Op POV left- hood pos
    - Op POV right- hood zero
    - Dr POV up/down- hood speeds
    - Repeat
    - Slip hood, than repeat [Didn't get to]

    - Shooter hood map (btwn fixed and map)
* ***********- adjust shooter speed map
- Investigate shooting during auto
* **Run hood to target at the beginning of shoot() to run all the time.
    - if time, add boolean method (supplier) to check if hood is at position. (WITH TIMEOUT TO GUARANTEE FINISH)
    - Do not run top rollers while intaking
        - This should prevent both rollers from running and jamming onto unpowered kicker during auto, drawing excess current.
    - Fix hood pulling amps during auto
        - A: switched from runHoodDown() to runHoodToZero() as runHoodDown() wasn't finishing or being interrupted.
            (as it was originally made for MANUALLY bringing hood down (Operator POV down))
- RIT STRENGTH
    - Pass by herding
    * ***- SUGGESTED PASS STRATEGY: Bump, center to wall, trench, repeat

- RIT GOAL
    - Pass by shooting
