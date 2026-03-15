Primary Objectives:

************- adjust shooter speed map
**TEAM REQ**: Move shoot pos closer to hub
**- change path to be even closer to hub on second rush to center, and stay in NZ

NEEDS TO TEST
        - Op POV left- hood pos
        - Op POV right- hood zero
        - Dr POV up/down- hood speeds
        - Repeat
        - Slip hood, than repeat

        - Shooter hood map (btwn fixed and map)


Low Priority:
    - "driveCurrentLimit": 15.0 in pathplanner?
    - Stop spotlessGradle from auto-formatting on build

MISC
    - Systems check list
    - Merge both TODO files in code

Do at Practice Field

- Test auto aim to hub
- Test shooter and hood maps (using hub april tag)






CURRENT STRENGTH
- Pass by herding
****- SUGGESTED PASS STRATEGY: Bump, center to wall, trench, repeat

CURRENT GOAL
- Pass by shooting
















Completed
- Investigate shooting during auto
***Run hood to target at the begininning of shoot() to run all the time.
        if time, add boolean method (supplier) to check if hood is at position. (WITH TIMOUT IF IT DONT WORKIE)
    - Do not run top rollers while intaking
        - This should both rollers from running and jamming onto unpowered kicker during auto, drawing excess current.
    - Fix hood pulling amps during auto
        THE FIX: switched from runHoodDown() to runHoodToZero() as runHoodDown() wasn't finishing or being interupted.
            (as it was origionally made for MANUALLY bringing hood down (Operator POV down))
- is drivetrain maxed out? A: No, kSpeedAt12Volts from 10 to 15 increased drive speed. reducing to 13.
