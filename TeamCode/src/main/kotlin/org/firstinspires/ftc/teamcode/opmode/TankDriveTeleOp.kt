package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.TankDrive

@TeleOp(name = "FGC: 2 Motor Tank Drive", group = "FGC")
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
