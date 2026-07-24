package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "FGC: Tank Drive", group = "FGC Vietnam")
class TankDriveTeleOp :
    SharedDriveTeleOp(DriveControlMode.TANK)
