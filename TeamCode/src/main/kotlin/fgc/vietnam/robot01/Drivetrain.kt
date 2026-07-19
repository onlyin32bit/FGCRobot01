package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.abs

internal data class DriveTelemetry(
    val requestedForward: Double,
    val limitedForward: Double,
    val requestedTurn: Double,
    val heading: Double,
    val targetHeading: Double,
    val headingError: Double,
    val headingCorrection: Double,
    val proportionalCorrection: Double,
    val integralCorrection: Double,
    val derivativeCorrection: Double,
    val yawRate: Double,
    val leftTargetVelocity: Double,
    val leftActualVelocity: Double,
    val rightTargetVelocity: Double,
    val rightActualVelocity: Double,
    val linearSpeedMmPerSecond: Double,
    val batteryVoltage: Double,
    val headingHoldEnabled: Boolean,
)

internal class Drivetrain(hardwareMap: HardwareMap) {
    private data class HeadingControl(
        val correction: Double = 0.0,
        val proportional: Double = 0.0,
        val integral: Double = 0.0,
        val derivative: Double = 0.0,
    )

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

    private val maxTicksPerSecond = minOf(
        leftMotor.motorType.achieveableMaxTicksPerSecond,
        rightMotor.motorType.achieveableMaxTicksPerSecond,
    )

    private val voltageSensor = hardwareMap.voltageSensor.iterator().next()

    private val imu = hardwareMap.get(IMU::class.java, IMU_NAME).apply {
        val orientation = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
        )

        check(initialize(IMU.Parameters(orientation))) {
            "Unable to initialize IMU '$IMU_NAME'"
        }
        resetYaw()
    }

    private var targetHeading = heading()
    private var limitedForward = 0.0
    private var lastDriveTimeNanos = System.nanoTime()
    private var headingHoldEnabled = true
    private var integralCorrection = 0.0

    fun drive(forward: Double, turn: Double): DriveTelemetry {
        val elapsedSeconds = loopElapsedSeconds()
        val smoothForward = limitForward(
            target = forward,
            elapsedSeconds = elapsedSeconds,
        )
        val currentHeading = heading()
        val holdingHeading = when {
            !headingHoldEnabled -> {
                targetHeading = currentHeading
                false
            }

            abs(turn) >= TURN_DEADBAND -> {
                targetHeading = currentHeading
                false
            }

            abs(smoothForward) < DRIVE_DEADBAND -> {
                targetHeading = currentHeading
                false
            }

            else -> true
        }

        val headingError = if (holdingHeading) {
            AngleUnit.normalizeDegrees(targetHeading - currentHeading)
        } else {
            0.0
        }
        val yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
            .zRotationRate
            .toDouble()
        val headingControl = if (holdingHeading) {
            headingControl(
                error = headingError,
                yawRate = yawRate,
                forward = smoothForward,
                elapsedSeconds = elapsedSeconds,
            )
        } else {
            resetHeadingControl()
            HeadingControl()
        }
        val correctedTurn = turn + headingControl.correction

        val left = smoothForward + correctedTurn
        val right = smoothForward - correctedTurn
        val scale = maxOf(1.0, abs(left), abs(right))
        val leftTargetVelocity = left / scale * maxTicksPerSecond
        val rightTargetVelocity = right / scale * maxTicksPerSecond

        setVelocity(
            left = leftTargetVelocity,
            right = rightTargetVelocity,
        )
        val leftActualVelocity = leftMotor.velocity
        val rightActualVelocity = rightMotor.velocity

        return DriveTelemetry(
            requestedForward = forward,
            limitedForward = smoothForward,
            requestedTurn = turn,
            heading = currentHeading,
            targetHeading = targetHeading,
            headingError = headingError,
            headingCorrection = headingControl.correction,
            proportionalCorrection = headingControl.proportional,
            integralCorrection = headingControl.integral,
            derivativeCorrection = headingControl.derivative,
            yawRate = yawRate,
            leftTargetVelocity = leftTargetVelocity,
            leftActualVelocity = leftActualVelocity,
            rightTargetVelocity = rightTargetVelocity,
            rightActualVelocity = rightActualVelocity,
            linearSpeedMmPerSecond = (
                leftActualVelocity + rightActualVelocity
            ) / 2.0 * DrivetrainConfig.millimetersPerEncoderTick,
            batteryVoltage = voltageSensor.voltage,
            headingHoldEnabled = headingHoldEnabled,
        )
    }

    fun toggleHeadingHold() {
        headingHoldEnabled = !headingHoldEnabled
        targetHeading = heading()
        resetHeadingControl()
    }

    fun stop() {
        limitedForward = 0.0
        resetHeadingControl()
        setVelocity(left = 0.0, right = 0.0)
    }

    private fun heading(): Double =
        imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

    private fun headingControl(
        error: Double,
        yawRate: Double,
        forward: Double,
        elapsedSeconds: Double,
    ): HeadingControl {
        val effectiveError = when {
            error > HEADING_TOLERANCE_DEGREES ->
                error - HEADING_TOLERANCE_DEGREES

            error < -HEADING_TOLERANCE_DEGREES ->
                error + HEADING_TOLERANCE_DEGREES

            else -> 0.0
        }
        val correctionLimit = minOf(
            MAX_HEADING_CORRECTION,
            abs(forward) * LOW_SPEED_CORRECTION_RATIO,
        )
        val proportional = effectiveError * HEADING_KP
        val derivative = -yawRate * HEADING_KD
        val proposedIntegral = (
            integralCorrection + effectiveError * HEADING_KI * elapsedSeconds
        ).coerceIn(
            minimumValue = -MAX_INTEGRAL_CORRECTION,
            maximumValue = MAX_INTEGRAL_CORRECTION,
        )
        val proposedCorrection = proportional + proposedIntegral + derivative

        val reversesExistingCorrection =
            effectiveError * proposedCorrection < 0.0
        if (
            abs(proposedCorrection) <= correctionLimit ||
            reversesExistingCorrection
        ) {
            integralCorrection = proposedIntegral
        }
        val correction = (
            proportional + integralCorrection + derivative
        ).coerceIn(
            minimumValue = -correctionLimit,
            maximumValue = correctionLimit,
        )

        return HeadingControl(
            correction = correction,
            proportional = proportional,
            integral = integralCorrection,
            derivative = derivative,
        )
    }

    private fun resetHeadingControl() {
        integralCorrection = 0.0
    }

    private fun loopElapsedSeconds(): Double {
        val now = System.nanoTime()
        val elapsedSeconds = ((now - lastDriveTimeNanos) / NANOS_PER_SECOND)
            .coerceIn(0.0, MAX_LOOP_TIME_SECONDS)
        lastDriveTimeNanos = now
        return elapsedSeconds
    }

    private fun limitForward(
        target: Double,
        elapsedSeconds: Double,
    ): Double {
        val rate = if (
            abs(target) < abs(limitedForward) ||
            target * limitedForward < 0.0
        ) {
            DECELERATION_PER_SECOND
        } else {
            ACCELERATION_PER_SECOND
        }
        val maximumChange = rate * elapsedSeconds
        limitedForward += (target - limitedForward).coerceIn(
            minimumValue = -maximumChange,
            maximumValue = maximumChange,
        )
        return limitedForward
    }

    private fun setVelocity(left: Double, right: Double) {
        leftMotor.velocity = left
        rightMotor.velocity = right
    }

    private fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name).apply {
        this.direction = direction
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        power = 0.0
    }

    private companion object {
        const val LEFT_MOTOR_NAME = "left_drive"
        const val RIGHT_MOTOR_NAME = "right_drive"
        const val IMU_NAME = "imu"

        const val DRIVE_DEADBAND = 0.05
        const val TURN_DEADBAND = 0.05
        const val HEADING_TOLERANCE_DEGREES = 0.35
        const val HEADING_KP = 0.012
        const val HEADING_KI = 0.004
        const val HEADING_KD = 0.0015
        const val MAX_INTEGRAL_CORRECTION = 0.03
        const val LOW_SPEED_CORRECTION_RATIO = 0.35
        const val MAX_HEADING_CORRECTION = 0.15

        const val ACCELERATION_PER_SECOND = 3.0
        const val DECELERATION_PER_SECOND = 5.0
        const val MAX_LOOP_TIME_SECONDS = 0.1
        const val NANOS_PER_SECOND = 1_000_000_000.0
    }
}
