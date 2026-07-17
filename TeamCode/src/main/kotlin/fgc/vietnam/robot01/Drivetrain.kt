package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

internal class Drivetrain(hardwareMap: HardwareMap) {
    private val leftMotor = motor(
        hardwareMap = hardwareMap,
        name = LEFT_MOTOR_NAME,
        direction = DcMotorSimple.Direction.REVERSE,
    )

    private val rightMotor = motor(
        hardwareMap = hardwareMap,
        name = RIGHT_MOTOR_NAME,
        direction = DcMotorSimple.Direction.FORWARD,
    )

    fun drive(forward: Double, turn: Double) {
        val left = forward + turn
        val right = forward - turn
        val scale = maxOf(1.0, abs(left), abs(right))

        setPower(left = left / scale, right = right / scale)
    }

    fun stop() = setPower(left = 0.0, right = 0.0)

    private fun setPower(left: Double, right: Double) {
        leftMotor.power = left
        rightMotor.power = right
    }

    private fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotor = hardwareMap.get(DcMotor::class.java, name).apply {
        this.direction = direction
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private companion object {
        const val LEFT_MOTOR_NAME = "left_drive"
        const val RIGHT_MOTOR_NAME = "right_drive"
    }
}
