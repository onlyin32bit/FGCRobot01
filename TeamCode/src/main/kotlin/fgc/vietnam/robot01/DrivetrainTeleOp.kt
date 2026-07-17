package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "FGC: Arcade Drive", group = "FGC Vietnam")
class DrivetrainTeleOp : OpMode() {
    private lateinit var drivetrain: Drivetrain

    override fun init() {
        drivetrain = Drivetrain(hardwareMap)
        telemetry.addData("Status", "Ready")
    }

    override fun loop() {
        val forward = -gamepad1.left_stick_y.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()

        drivetrain.drive(forward = forward, turn = turn)
        telemetry.addData("Drive", "forward=%.2f turn=%.2f", forward, turn)
    }

    override fun stop() = drivetrain.stop()
}
