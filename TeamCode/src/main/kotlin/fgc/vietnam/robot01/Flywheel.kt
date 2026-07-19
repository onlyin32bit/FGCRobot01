package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs
import kotlin.math.max

internal data class FlywheelMotorTelemetry(
    val rpm: Double,
    val velocity: Double,
    val currentAmps: Double,
)

internal data class FlywheelTelemetry(
    val enabled: Boolean,
    val atSpeed: Boolean,
    val targetRpm: Double,
    val targetVelocity: Double,
    val primaryMotor: FlywheelMotorTelemetry,
    val secondaryMotor: FlywheelMotorTelemetry,
    val shaftRpm: Double,
    val rpmDifference: Double,
    val surfaceSpeedMetersPerSecond: Double,
)

internal class Flywheel(hardwareMap: HardwareMap) {
    private val primaryMotor = motor(
        hardwareMap = hardwareMap,
        name = FlywheelConfig.PRIMARY_MOTOR_NAME,
        direction = DcMotorSimple.Direction.REVERSE,
    )

    private val secondaryMotor = motor(
        hardwareMap = hardwareMap,
        name = FlywheelConfig.SECONDARY_MOTOR_NAME,
        direction = DcMotorSimple.Direction.FORWARD,
    )

    private var enabled = false
    private var targetRpm = FlywheelConfig.DEFAULT_RPM

    fun toggle() {
        enabled = !enabled
    }

    fun increaseSpeed() {
        targetRpm = (targetRpm + FlywheelConfig.RPM_STEP)
            .coerceAtMost(FlywheelConfig.MAX_RPM)
    }

    fun decreaseSpeed() {
        targetRpm = (targetRpm - FlywheelConfig.RPM_STEP)
            .coerceAtLeast(FlywheelConfig.MIN_RPM)
    }

    fun update(): FlywheelTelemetry {
        val commandedRpm = if (enabled) targetRpm else 0.0
        val targetVelocity =
            FlywheelConfig.rpmToTicksPerSecond(commandedRpm)
        primaryMotor.velocity = targetVelocity
        secondaryMotor.velocity = targetVelocity

        val primaryState = motorTelemetry(primaryMotor)
        val secondaryState = motorTelemetry(secondaryMotor)
        val shaftRpm = (primaryState.rpm + secondaryState.rpm) / 2.0
        val allowedErrorRpm = max(
            MIN_READY_ERROR_RPM,
            targetRpm * READY_ERROR_RATIO,
        )

        return FlywheelTelemetry(
            enabled = enabled,
            atSpeed = enabled &&
                abs(primaryState.rpm - targetRpm) <= allowedErrorRpm &&
                abs(secondaryState.rpm - targetRpm) <= allowedErrorRpm,
            targetRpm = targetRpm,
            targetVelocity = targetVelocity,
            primaryMotor = primaryState,
            secondaryMotor = secondaryState,
            shaftRpm = shaftRpm,
            rpmDifference = abs(primaryState.rpm - secondaryState.rpm),
            surfaceSpeedMetersPerSecond =
                FlywheelConfig.rpmToSurfaceSpeedMetersPerSecond(shaftRpm),
        )
    }

    fun stop() {
        enabled = false
        primaryMotor.velocity = 0.0
        secondaryMotor.velocity = 0.0
    }

    private fun motorTelemetry(motor: DcMotorEx): FlywheelMotorTelemetry {
        val velocity = motor.velocity
        return FlywheelMotorTelemetry(
            rpm = FlywheelConfig.ticksPerSecondToRpm(velocity),
            velocity = velocity,
            currentAmps = motor.getCurrent(CurrentUnit.AMPS),
        )
    }

    private fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name).apply {
        this.direction = direction
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        power = 0.0
    }

    private companion object {
        const val READY_ERROR_RATIO = 0.05
        const val MIN_READY_ERROR_RPM = 100.0
    }
}
