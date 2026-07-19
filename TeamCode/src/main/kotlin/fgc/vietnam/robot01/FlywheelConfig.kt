package fgc.vietnam.robot01

import kotlin.math.PI

internal object FlywheelConfig {
    const val PRIMARY_MOTOR_NAME = "flywheel"
    const val SECONDARY_MOTOR_NAME = "flywheel_2"

    // Motor rotations per wheel rotation.
    const val GEAR_REDUCTION = 1.0
    const val MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0
    const val WHEEL_DIAMETER_MM = 90.0

    const val DEFAULT_RPM = 1_500.0
    const val RPM_STEP = 250.0
    const val MIN_RPM = 500.0
    const val MAX_RPM = 5_500.0

    val encoderTicksPerWheelRevolution =
        MOTOR_ENCODER_TICKS_PER_REVOLUTION * GEAR_REDUCTION

    fun rpmToTicksPerSecond(rpm: Double) =
        rpm * encoderTicksPerWheelRevolution / 60.0

    fun ticksPerSecondToRpm(ticksPerSecond: Double) =
        ticksPerSecond * 60.0 / encoderTicksPerWheelRevolution

    fun rpmToSurfaceSpeedMetersPerSecond(rpm: Double) =
        rpm / 60.0 * PI * WHEEL_DIAMETER_MM / 1_000.0
}
