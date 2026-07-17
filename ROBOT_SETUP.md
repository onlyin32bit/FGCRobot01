# FGC Robot 01 setup

This project is based on the FTC Robot Controller SDK 11.1 and is ready to
deploy to a REV Control Hub from Android Studio.

## Control Hub configuration

Create or edit the active robot configuration on the Driver Station:

| Control Hub port | Device type | Configuration name |
| --- | --- | --- |
| Motor port 0 | Your REV-compatible DC motor | `left_drive` |
| Motor port 1 | Your REV-compatible DC motor | `right_drive` |

The configuration names are case-sensitive and must exactly match the names
above. The motor ports can be changed, but the names must stay the same unless
they are also changed in `TankDrive.kt`.

## Driving

1. Connect the Driver Station to the Control Hub.
2. Select `FGC: 2 Motor Tank Drive` from the TeleOp list.
3. Press INIT, then START.
4. Use gamepad 1's left stick for the left wheel and right stick for the right
   wheel.

Test with the wheels raised off the floor first. Both sticks pushed forward
should drive both wheels forward. If a wheel runs backward, swap that motor's
`FORWARD`/`REVERSE` direction in `TankDrive.kt`.

Robot code belongs in:

`TeamCode/src/main/kotlin/org/firstinspires/ftc/teamcode/`
