package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

internal class TankDrive(hardwareMap: HardwareMap) {
    private val leftMotor = hardwareMap.driveMotor(
        name = LEFT_MOTOR_NAME,
        direction = DcMotorSimple.Direction.REVERSE,
    )

    private val rightMotor = hardwareMap.driveMotor(
        name = RIGHT_MOTOR_NAME,
        direction = DcMotorSimple.Direction.FORWARD,
    )

    fun setPower(left: Double, right: Double) {
        leftMotor.power = left
        rightMotor.power = right
    }

    fun stop() = setPower(left = 0.0, right = 0.0)

    private companion object {
        const val LEFT_MOTOR_NAME = "left_drive"
        const val RIGHT_MOTOR_NAME = "right_drive"
    }
}

private fun HardwareMap.driveMotor(
    name: String,
    direction: DcMotorSimple.Direction,
): DcMotor = get(DcMotor::class.java, name).apply {
    this.direction = direction
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    power = 0.0
}
