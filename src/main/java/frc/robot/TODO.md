High Priority:
--------------
V3 Integration:
--------------
### On Deck
- Make hood go down faster after shooting
- timeout for slapdown curl while shooting
- increase slow speed for drive
- Save Limelight pipeline
- Shooter turnToTarget PID increase
- Auto
    - 2.5 cycle auto (with can range)
        - First Pass:
            - hook back around to intake more on way to bump
            - New combined path to bump OR set goal end-state velocity
        - Reduce shooter timeout
    - Test Warm Up Shooter named command to see if it is working now that we coast the shooter
- Deploy
    - Log peak stator/supply current for
        - forward
        - reverse
        - add current graph tab in Elastic
        - Stow while shooting state
- Hopper
    - Hopper empty detection
        - current based and can range
- Intake
    - Duty cycle intake rollers
    - Check follower, if OK use velocity
    - Increase duty cycle if velocity below threshold
    - Test intaking at depot
- Hood
    - Run hood to 100, then run to zero to speed up zeroing
READY TO TEST
    - implement CANRange - Jeff


===============================================

-
- Hood:
    - Investigate why it either works or not at all between code deploys
    - Hood zero trigger
- Limit ramp-rate from OFF to IDLE speed, but not from IDLE to ON
    - Tune motor configs to prep to shoot faster

- Intake
    - Ideas for smoother slapdown functions
        - See if kG is suitable: kG increase as angle approaches deploy intake to slow down
        - Simplest: Only apply output up until angle exceeds pre-defined drop-point angle; let gravity do the rest
    - Handle chain skipping on deploy. since it pushes 0 (stow) out more, it prevents reaching DIO limit which prevents 0'ing
        - Idea: constant unsubstantial output in appropiate direction until DIO triggered. (Above tasks may fix this anyway)
        - Idea: clamp angle to never fall outside of 0.0 and DOWN range (by manipulating offset)
    - Verify zeroing only happens when slapdown is up.

- Autos
    - First rush to NZ should have rotation slightly biased towards AZ to protect intake in event of bot-to-bot collision

- Loop overruns: look up in "Visual VM" tool in WPILib track down issue

#### Code Refactor:
- Consider renaming rollers and deploy to better reflect V3
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

- Calculate PP MOI using sysID (Currently a rough estimate)
- Ask Nick W for TPU tread COF
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
- Ask CAD for COF of the 3D printed swerve treads


Albany Robot Changes:
---------------------
- Slapdown intake rather than rack and pinion
    - May or may not have a powered reaction bar
- Removing top rollers
- Verticle extendable hopper (Similar to 1678)
- Full length shooter
    - Full length kicker
    - Limelight has to move
- Shooter rotates 180 degrees
- Possibly maybe an L1 climber







<br><br><br><br><br><br><br><br>







<br><br><br><br><br><br><br><br>
Completed
---------
 - look at peak forward/rev torque current slapdown
- shorten fuel launching arc at pose where auto shoots
- Test if deploy seed pos if down with triggers
    - A: zero encoder works, but not seeding non-zero position. Is this a trigger issue or encoder-side issue?
- Shooter coast to idle
    - Shooter modes in applyOutputs
- V3 Intake:
    - Deploy function without up/down sensors
    - Consider adding DIO invertion (refer to ShooterIOKraken)
    - Deploy: negative output stows intake up
        - Limit Extent: DIO <?>
        - Limit Stow: DIO <?>
        - Stator: 20
        - Supply: 12
        - Idle: Coeast

    - Intake: only right is inverted (clockwise positive)
        - Stator: 120
        - Supply: 40
        - Idle: Coast
- Autos:
    - Auto "Bottom Rush to Hub" is still hitting wall, worsening after each trench pass. Odometry loss is likely aggravated due to camara not seeing April Tags at all through auto. Path must be edited to work for V3 cam placement/shooter reorientation.
    - Investigate shoot from NZ passing map (can test via sim)
    - Investigate why shoot named command is causing odd rotation during bottom bump auto? is it v2 or v3?

- Investigate why Operator intake only buttons (POV up/down) were not working
    - A: Intake idle was an instant command, so its default command (set to stop intake) was likely running directly after. Since some command groups rely on it being an instant command (ends instantly), we wrapped it in a RepeatCommand only where Op' POV up calls it.
- "driveCurrentLimit": 15.0 in pathplanner?
    - A: PP app weight was very light
- Verify whether PP_CONFIG for PathPlanner overrides our app settings. Consider tuning/matching configs with app
    - A: Updated values in code and set app to match. MOI roughly calculated & left COF at default
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
