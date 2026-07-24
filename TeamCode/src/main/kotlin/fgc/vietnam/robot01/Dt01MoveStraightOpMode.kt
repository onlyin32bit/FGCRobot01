package fgc.vietnam.robot01

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import java.io.IOException
import kotlin.math.abs

@TeleOp(name = "FGC Test: DT01 Move Straight", group = "FGC Tests")
class Dt01MoveStraightOpMode : Dt01MoveStraightBaseOpMode(
    scenarioId = "DT01",
    driveCommand = 0.30,
)

@TeleOp(name = "FGC Test: DT01_1 Move Straight (0.15)", group = "FGC Tests")
class Dt011MoveStraightOpMode : Dt01MoveStraightBaseOpMode(
    scenarioId = "DT01_1",
    driveCommand = 0.15,
)

@TeleOp(name = "FGC Test: DT01_2 Move Straight (1.00)", group = "FGC Tests")
class Dt012MoveStraightOpMode : Dt01MoveStraightBaseOpMode(
    scenarioId = "DT01_2",
    driveCommand = 1.00,
)

abstract class Dt01MoveStraightBaseOpMode protected constructor(
    private val scenarioId: String,
    private val driveCommand: Double,
) : OpMode() {
    init {
        require(driveCommand in 0.0..1.0) {
            "DT01 drive command must be between 0 and 1"
        }
    }

    private enum class Phase {
        WAITING,
        RUNNING,
        BRAKING,
        COMPLETE,
        ABORTED,
    }

    private enum class TestDirection(
        val sign: Double,
        val label: String,
    ) {
        FORWARD(sign = 1.0, label = "FORWARD"),
        BACKWARD(sign = -1.0, label = "BACKWARD"),
    }

    private data class MotorTargets(
        val leftTicksPerSecond: Double = 0.0,
        val rightTicksPerSecond: Double = 0.0,
    )

    private lateinit var leftMotor: DcMotorEx
    private lateinit var rightMotor: DcMotorEx
    private lateinit var imu: IMU
    private lateinit var voltageSensor: VoltageSensor

    private var logger: ExperimentLogger? = null
    private var logFileName: String? = null
    private var loggerError: String? = null
    private var finalRowCount = 0

    private var phase = Phase.WAITING
    private var direction: TestDirection? = null
    private var runStartNanos = 0L
    private var phaseStartNanos = 0L
    private var lastLoopNanos = 0L
    private var sampleIndex = 0
    private var leftStartTicks = 0
    private var rightStartTicks = 0
    private var targetHeadingDegrees = 0.0

    private var previousForwardButton = false
    private var previousBackwardButton = false
    private val pendingEvents = mutableListOf<ExperimentEvent>()

    override fun init() {
        leftMotor = driveMotor(
            hardwareMap = hardwareMap,
            name = LEFT_MOTOR_NAME,
            direction = DcMotorSimple.Direction.REVERSE,
        )
        rightMotor = driveMotor(
            hardwareMap = hardwareMap,
            name = RIGHT_MOTOR_NAME,
            direction = DcMotorSimple.Direction.FORWARD,
        )
        imu = hardwareMap.get(IMU::class.java, IMU_NAME).apply {
            val orientation = RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
            )
            check(initialize(IMU.Parameters(orientation))) {
                "Unable to initialize IMU '$IMU_NAME'"
            }
            resetYaw()
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        telemetry.addData(
            scenarioId,
            "Align robot on a clear 2.5 m course | power=%.2f",
            driveCommand,
        )
        telemetry.addData("A", "Start FORWARD 2 m")
        telemetry.addData("Y", "Start BACKWARD 2 m")
        telemetry.addData("B", "Abort")
    }

    override fun start() {
        val now = System.nanoTime()
        runStartNanos = now
        phaseStartNanos = now
        lastLoopNanos = now
        pendingEvents += ExperimentEvent.LOG_START

        try {
            logger = ExperimentLogger.create(
                prefix = "${scenarioId}_move_straight",
                context = ExperimentRunContext(
                    scenarioId = scenarioId,
                    configurationId = "${scenarioId}_P_ONLY_V1",
                ),
                scenarioColumns = SCENARIO_COLUMNS,
            )
            logFileName = logger?.fileName
        } catch (exception: IOException) {
            loggerError = exception.message ?: exception.javaClass.simpleName
            abort(ExperimentEvent.FAULT)
        }
    }

    override fun loop() {
        val now = System.nanoTime()
        val loopDtMilliseconds =
            (now - lastLoopNanos) / NANOS_PER_MILLISECOND
        lastLoopNanos = now

        if (gamepad1.b && phase !in TERMINAL_PHASES) {
            abort(ExperimentEvent.ABORT)
        }

        handleStartButtons(now)

        val selectedDirection = direction
        val leftPosition = leftMotor.currentPosition
        val rightPosition = rightMotor.currentPosition
        val directionSign = selectedDirection?.sign ?: 0.0
        val leftDistanceMm = directionSign *
            (leftPosition - leftStartTicks) *
            DrivetrainConfig.millimetersPerEncoderTick
        val rightDistanceMm = directionSign *
            (rightPosition - rightStartTicks) *
            DrivetrainConfig.millimetersPerEncoderTick
        val progressMm = (leftDistanceMm + rightDistanceMm) / 2.0
        val currentHeading =
            imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
        val yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
            .zRotationRate
            .toDouble()
        val headingError = if (phase == Phase.RUNNING) {
            AngleUnit.normalizeDegrees(
                targetHeadingDegrees - currentHeading,
            )
        } else {
            0.0
        }
        val proportionalCorrection = if (phase == Phase.RUNNING) {
            simpleHeadingCorrection(headingError)
        } else {
            0.0
        }

        advanceTest(
            now = now,
            progressMm = progressMm,
            leftVelocity = leftMotor.velocity,
            rightVelocity = rightMotor.velocity,
        )

        val targets = applyMotorCommand(proportionalCorrection)
        val leftVelocity = leftMotor.velocity
        val rightVelocity = rightMotor.velocity
        val leftCurrent = leftMotor.getCurrent(CurrentUnit.AMPS)
        val rightCurrent = rightMotor.getCurrent(CurrentUnit.AMPS)
        val stopped = abs(leftVelocity) <= STOPPED_TICKS_PER_SECOND &&
            abs(rightVelocity) <= STOPPED_TICKS_PER_SECOND
        val eventText = ExperimentLogger.events(*pendingEvents.toTypedArray())
        pendingEvents.clear()

        val wroteRow = logger?.write(
            sample = ExperimentSample(
                sampleIndex = sampleIndex,
                timeS = secondsBetween(runStartNanos, now),
                loopDtMs = loopDtMilliseconds,
                event = eventText,
                batteryV = voltageSensor.voltage,
            ),
            scenarioValues = mapOf(
                "scenario_phase" to phase.name,
                "scenario_direction" to
                    (selectedDirection?.label ?: "NONE"),
                "scenario_controller_version" to CONTROLLER_VERSION,
                "scenario_target_distance_mm" to TARGET_DISTANCE_MM,
                "scenario_left_distance_mm" to leftDistanceMm,
                "scenario_right_distance_mm" to rightDistanceMm,
                "scenario_progress_mm" to progressMm,
                "scenario_remaining_mm" to
                    (TARGET_DISTANCE_MM - progressMm),
                "scenario_distance_difference_mm" to
                    abs(leftDistanceMm - rightDistanceMm),
                "scenario_drive_command" to if (phase == Phase.RUNNING) {
                    (selectedDirection?.sign ?: 0.0) * driveCommand
                } else {
                    0.0
                },
                "scenario_target_heading_deg" to targetHeadingDegrees,
                "scenario_heading_deg" to currentHeading,
                "scenario_heading_error_deg" to headingError,
                "scenario_heading_kp" to HEADING_KP,
                "scenario_heading_p" to proportionalCorrection,
                "scenario_yaw_rate_deg_s" to yawRate,
                "scenario_left_target_tps" to targets.leftTicksPerSecond,
                "scenario_right_target_tps" to targets.rightTicksPerSecond,
                "scenario_left_actual_tps" to leftVelocity,
                "scenario_right_actual_tps" to rightVelocity,
                "scenario_left_position_ticks" to leftPosition,
                "scenario_right_position_ticks" to rightPosition,
                "scenario_left_current_a" to leftCurrent,
                "scenario_right_current_a" to rightCurrent,
                "scenario_stopped" to stopped,
                "scenario_gear_reduction" to
                    DrivetrainConfig.GEAR_REDUCTION,
                "scenario_wheel_diameter_mm" to
                    DrivetrainConfig.WHEEL_DIAMETER_MM,
                "scenario_track_width_mm" to
                    DrivetrainConfig.TRACK_WIDTH_MM,
            ),
        ) ?: false
        sampleIndex += 1

        if (!wroteRow && phase !in TERMINAL_PHASES) {
            loggerError = logger?.errorMessage ?: "Logger unavailable"
            abort(ExperimentEvent.FAULT)
        }

        showTelemetry(
            progressMm = progressMm,
            leftDistanceMm = leftDistanceMm,
            rightDistanceMm = rightDistanceMm,
            headingError = headingError,
            proportionalCorrection = proportionalCorrection,
            leftVelocity = leftVelocity,
            rightVelocity = rightVelocity,
            leftCurrent = leftCurrent,
            rightCurrent = rightCurrent,
        )

        if (phase in TERMINAL_PHASES) closeLogger()
    }

    override fun stop() {
        stopMotors()
        if (phase !in TERMINAL_PHASES) {
            writeTerminalEvent(ExperimentEvent.ABORT)
        }
        closeLogger()
    }

    private fun handleStartButtons(now: Long) {
        val forwardPressed = gamepad1.a && !previousForwardButton
        val backwardPressed = gamepad1.y && !previousBackwardButton

        if (
            phase == Phase.WAITING &&
            forwardPressed.xor(backwardPressed)
        ) {
            beginTest(
                selectedDirection = if (forwardPressed) {
                    TestDirection.FORWARD
                } else {
                    TestDirection.BACKWARD
                },
                now = now,
            )
        }

        previousForwardButton = gamepad1.a
        previousBackwardButton = gamepad1.y
    }

    private fun beginTest(
        selectedDirection: TestDirection,
        now: Long,
    ) {
        direction = selectedDirection
        leftStartTicks = leftMotor.currentPosition
        rightStartTicks = rightMotor.currentPosition
        targetHeadingDegrees =
            imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
        phase = Phase.RUNNING
        phaseStartNanos = now
        pendingEvents += ExperimentEvent.START
        pendingEvents += ExperimentEvent.COMMAND_START
    }

    private fun advanceTest(
        now: Long,
        progressMm: Double,
        leftVelocity: Double,
        rightVelocity: Double,
    ) {
        val phaseSeconds = secondsBetween(phaseStartNanos, now)
        when (phase) {
            Phase.RUNNING -> when {
                progressMm >= TARGET_DISTANCE_MM -> {
                    phase = Phase.BRAKING
                    phaseStartNanos = now
                    pendingEvents += ExperimentEvent.TARGET_REACHED
                    pendingEvents += ExperimentEvent.COMMAND_STOP
                }

                phaseSeconds >= RUN_TIMEOUT_SECONDS ->
                    abort(ExperimentEvent.FAULT)

                else -> Unit
            }

            Phase.BRAKING -> {
                val stopped =
                    abs(leftVelocity) <= STOPPED_TICKS_PER_SECOND &&
                        abs(rightVelocity) <= STOPPED_TICKS_PER_SECOND
                when {
                    stopped && phaseSeconds >= MIN_BRAKING_SECONDS -> {
                        phase = Phase.COMPLETE
                        pendingEvents += ExperimentEvent.STOPPED
                        pendingEvents += ExperimentEvent.END
                    }

                    phaseSeconds >= BRAKING_TIMEOUT_SECONDS ->
                        abort(ExperimentEvent.FAULT)

                    else -> Unit
                }
            }

            else -> Unit
        }
    }

    /** DT01 baseline controller: proportional heading correction only. */
    private fun simpleHeadingCorrection(headingErrorDegrees: Double): Double =
        (headingErrorDegrees * HEADING_KP).coerceIn(
            minimumValue = -MAX_HEADING_CORRECTION,
            maximumValue = MAX_HEADING_CORRECTION,
        )

    private fun applyMotorCommand(
        proportionalCorrection: Double,
    ): MotorTargets {
        if (phase != Phase.RUNNING) {
            stopMotors()
            return MotorTargets()
        }

        val baseCommand = direction?.sign?.times(driveCommand) ?: 0.0
        val leftCommand =
            (baseCommand + proportionalCorrection).coerceIn(-1.0, 1.0)
        val rightCommand =
            (baseCommand - proportionalCorrection).coerceIn(-1.0, 1.0)
        val maximumVelocity = minOf(
            leftMotor.motorType.achieveableMaxTicksPerSecond,
            rightMotor.motorType.achieveableMaxTicksPerSecond,
        )
        val targets = MotorTargets(
            leftTicksPerSecond = leftCommand * maximumVelocity,
            rightTicksPerSecond = rightCommand * maximumVelocity,
        )
        leftMotor.velocity = targets.leftTicksPerSecond
        rightMotor.velocity = targets.rightTicksPerSecond
        return targets
    }

    private fun abort(event: ExperimentEvent) {
        if (phase in TERMINAL_PHASES) return
        phase = Phase.ABORTED
        pendingEvents += event
        pendingEvents += ExperimentEvent.END
        stopMotors()
    }

    private fun showTelemetry(
        progressMm: Double,
        leftDistanceMm: Double,
        rightDistanceMm: Double,
        headingError: Double,
        proportionalCorrection: Double,
        leftVelocity: Double,
        rightVelocity: Double,
        leftCurrent: Double,
        rightCurrent: Double,
    ) {
        telemetry.addData(
            scenarioId,
            "%s | power=%.2f | P-only baseline",
            phase.name,
            driveCommand,
        )
        telemetry.addData(
            "Direction",
            direction?.label ?: "A=FORWARD | Y=BACKWARD",
        )
        telemetry.addData(
            "Distance",
            "%.0f/%.0f mm | L=%.0f R=%.0f",
            progressMm,
            TARGET_DISTANCE_MM,
            leftDistanceMm,
            rightDistanceMm,
        )
        telemetry.addData(
            "Heading P",
            "error=%+.2f deg correction=%+.3f",
            headingError,
            proportionalCorrection,
        )
        telemetry.addData(
            "Velocity",
            "L=%.0f R=%.0f ticks/s",
            leftVelocity,
            rightVelocity,
        )
        telemetry.addData(
            "Current",
            "L=%.2f A R=%.2f A",
            leftCurrent,
            rightCurrent,
        )
        telemetry.addData(
            "Log",
            "%s | rows=%d%s",
            logger?.fileName ?: logFileName ?: "unavailable",
            logger?.rowCount ?: finalRowCount,
            loggerError?.let { " | ERROR: $it" }.orEmpty(),
        )
        if (phase !in TERMINAL_PHASES) {
            telemetry.addData("Stop", "B aborts immediately")
        }
    }

    private fun writeTerminalEvent(event: ExperimentEvent) {
        logger?.write(
            sample = ExperimentSample(
                sampleIndex = sampleIndex,
                timeS = if (runStartNanos == 0L) {
                    0.0
                } else {
                    secondsBetween(runStartNanos, System.nanoTime())
                },
                event = ExperimentLogger.events(event, ExperimentEvent.END),
                batteryV = voltageSensor.voltage,
            ),
            scenarioValues = SCENARIO_COLUMNS.associateWith { null },
        )
        sampleIndex += 1
    }

    private fun closeLogger() {
        finalRowCount = logger?.rowCount ?: finalRowCount
        logger?.close()
        loggerError = loggerError ?: logger?.errorMessage
        logger = null
    }

    private fun stopMotors() {
        leftMotor.velocity = 0.0
        rightMotor.velocity = 0.0
        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }

    private fun driveMotor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name).apply {
        this.direction = direction
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        power = 0.0
    }

    private fun secondsBetween(startNanos: Long, endNanos: Long): Double =
        (endNanos - startNanos) / NANOS_PER_SECOND

    private companion object {
        const val LEFT_MOTOR_NAME = "left_drive"
        const val RIGHT_MOTOR_NAME = "right_drive"
        const val IMU_NAME = "imu"
        const val CONTROLLER_VERSION = "P_ONLY_V1"

        const val TARGET_DISTANCE_MM = 2_000.0
        const val HEADING_KP = 0.012
        const val MAX_HEADING_CORRECTION = 0.15

        const val STOPPED_TICKS_PER_SECOND = 20.0
        const val MIN_BRAKING_SECONDS = 0.15
        const val BRAKING_TIMEOUT_SECONDS = 2.0
        const val RUN_TIMEOUT_SECONDS = 12.0

        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val NANOS_PER_MILLISECOND = 1_000_000.0

        val TERMINAL_PHASES = setOf(Phase.COMPLETE, Phase.ABORTED)

        val SCENARIO_COLUMNS = listOf(
            "scenario_phase",
            "scenario_direction",
            "scenario_controller_version",
            "scenario_target_distance_mm",
            "scenario_left_distance_mm",
            "scenario_right_distance_mm",
            "scenario_progress_mm",
            "scenario_remaining_mm",
            "scenario_distance_difference_mm",
            "scenario_drive_command",
            "scenario_target_heading_deg",
            "scenario_heading_deg",
            "scenario_heading_error_deg",
            "scenario_heading_kp",
            "scenario_heading_p",
            "scenario_yaw_rate_deg_s",
            "scenario_left_target_tps",
            "scenario_right_target_tps",
            "scenario_left_actual_tps",
            "scenario_right_actual_tps",
            "scenario_left_position_ticks",
            "scenario_right_position_ticks",
            "scenario_left_current_a",
            "scenario_right_current_a",
            "scenario_stopped",
            "scenario_gear_reduction",
            "scenario_wheel_diameter_mm",
            "scenario_track_width_mm",
        )
    }
}
