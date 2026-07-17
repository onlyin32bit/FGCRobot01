package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "FGC: 2 Motor Tank Drive", group = "FGC Vietnam")
class TankDriveTeleOp : OpMode() {
    private lateinit var drive: TankDrive

    override fun init() {
        drive = TankDrive(hardwareMap)

        telemetry.apply {
            addLine("Tank drive ready")
            addLine("Left stick = left motor, right stick = right motor")
        }
    }

    override fun loop() {
        val leftPower = -gamepad1.left_stick_y.toDouble()
        val rightPower = -gamepad1.right_stick_y.toDouble()

        drive.setPower(left = leftPower, right = rightPower)

        telemetry.apply {
            addData("Left power", "%.2f", leftPower)
            addData("Right power", "%.2f", rightPower)
        }
    }

    override fun stop() = drive.stop()
}
