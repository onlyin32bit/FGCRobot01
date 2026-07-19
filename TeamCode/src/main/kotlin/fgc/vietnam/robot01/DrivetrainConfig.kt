package fgc.vietnam.robot01

import kotlin.math.PI

internal object DrivetrainConfig {
    // Motor rotations per wheel rotation. Change this when the gearing changes.
    const val GEAR_REDUCTION = 13.0975

    const val MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0
    const val WHEEL_DIAMETER_MM = 90.0
    const val TRACK_WIDTH_MM = 423.0

    val encoderTicksPerWheelRevolution =
        MOTOR_ENCODER_TICKS_PER_REVOLUTION * GEAR_REDUCTION

    val millimetersPerEncoderTick =
        PI * WHEEL_DIAMETER_MM / encoderTicksPerWheelRevolution
}
