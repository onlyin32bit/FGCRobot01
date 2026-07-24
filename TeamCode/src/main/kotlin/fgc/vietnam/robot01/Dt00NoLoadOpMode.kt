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
import kotlin.math.max
import kotlin.math.sqrt

@TeleOp(name = "FGC Test: DT00 No-Load", group = "FGC Tests")
class Dt00NoLoadOpMode : OpMode() {
    private enum class Phase {
        WAITING,
        IDLE_FORWARD,
        DRIVE_FORWARD,
        COAST_FORWARD,
        IDLE_REVERSE,
        DRIVE_REVERSE,
        COAST_REVERSE,
        COMPLETE,
        ABORTED,
    }

    private class RollingDeviation(private val capacity: Int) {
        private val values = DoubleArray(capacity)
        private var count = 0
        private var nextIndex = 0
        private var sum = 0.0
        private var sumOfSquares = 0.0

        fun add(value: Double) {
            if (count == capacity) {
                val removed = values[nextIndex]
                sum -= removed
                sumOfSquares -= removed * removed
            } else {
                count += 1
            }

            values[nextIndex] = value
            nextIndex = (nextIndex + 1) % capacity
            sum += value
            sumOfSquares += value * value
        }

        fun rms(): Double {
            if (count < 2) return 0.0
            val mean = sum / count
            return sqrt(max(0.0, sumOfSquares / count - mean * mean))
        }

        fun reset() {
            count = 0
            nextIndex = 0
            sum = 0.0
            sumOfSquares = 0.0
        }
    }

    private lateinit var leftMotor: DcMotorEx
    private lateinit var rightMotor: DcMotorEx
    private lateinit var imu: IMU
    private lateinit var voltageSensor: VoltageSensor

    private val leftRpmDeviation = RollingDeviation(RMS_WINDOW_SAMPLES)
    private val rightRpmDeviation = RollingDeviation(RMS_WINDOW_SAMPLES)
    private val angularXDeviation = RollingDeviation(RMS_WINDOW_SAMPLES)
    private val angularYDeviation = RollingDeviation(RMS_WINDOW_SAMPLES)
    private val angularZDeviation = RollingDeviation(RMS_WINDOW_SAMPLES)

    private var logger: ExperimentLogger? = null
    private var logFileName: String? = null
    private var finalRowCount = 0
    private var loggerError: String? = null
    private var phase = Phase.WAITING
    private var runStartNanos = 0L
    private var phaseStartNanos = 0L
    private var lastLoopNanos = 0L
    private var sample = 0
    private var targetTicksPerSecond = 0.0
    private var previousStartButton = false
    private var motionDetected = false
    private var settledDetected = false
    private var coastStoppedDetected = false
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
        }
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        telemetry.addData("DT00", "Raise and secure robot before START")
        telemetry.addData("Controls", "A begin | B abort")
        telemetry.addData(
            "Profile",
            "1 s idle | %.0f%% for %.1f s | %.1f s coast | reverse=%s",
            STANDARD_COMMAND * 100.0,
            DRIVE_DURATION_SECONDS,
            COAST_DURATION_SECONDS,
            if (TEST_REVERSE) "ON" else "OFF",
        )
    }

    override fun start() {
        val now = System.nanoTime()
        runStartNanos = now
        phaseStartNanos = now
        lastLoopNanos = now
        phase = Phase.WAITING
        pendingEvents += ExperimentEvent.LOG_START

        try {
            logger = ExperimentLogger.create(
                prefix = "DT00_no_load",
                context = ExperimentRunContext(
                    scenarioId = SCENARIO_ID,
                    configurationId = CONFIGURATION_ID,
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
        val timeSeconds = secondsBetween(runStartNanos, now)
        val loopDtMilliseconds =
            (now - lastLoopNanos) / NANOS_PER_MILLISECOND
        lastLoopNanos = now

        if (gamepad1.b && phase !in TERMINAL_PHASES) {
            abort(ExperimentEvent.ABORT)
        }

        val startPressed = gamepad1.a
        if (
            phase == Phase.WAITING &&
            startPressed &&
            !previousStartButton
        ) {
            transitionTo(Phase.IDLE_FORWARD, ExperimentEvent.START, now)
        }
        previousStartButton = startPressed

        advanceProfile(now)
        applyMotorCommand()

        val leftVelocity = leftMotor.velocity
        val rightVelocity = rightMotor.velocity
        val leftRpm = ticksPerSecondToWheelRpm(leftVelocity)
        val rightRpm = ticksPerSecondToWheelRpm(rightVelocity)
        val leftCurrent = leftMotor.getCurrent(CurrentUnit.AMPS)
        val rightCurrent = rightMotor.getCurrent(CurrentUnit.AMPS)
        val angularVelocity =
            imu.getRobotAngularVelocity(AngleUnit.DEGREES)

        updateRollingMeasurements(
            leftRpm = leftRpm,
            rightRpm = rightRpm,
            angularX = angularVelocity.xRotationRate.toDouble(),
            angularY = angularVelocity.yRotationRate.toDouble(),
            angularZ = angularVelocity.zRotationRate.toDouble(),
        )

        val angularXRms = angularXDeviation.rms()
        val angularYRms = angularYDeviation.rms()
        val angularZRms = angularZDeviation.rms()
        val angularVibrationRms =
            sqrt(
                angularXRms * angularXRms +
                    angularYRms * angularYRms +
                    angularZRms * angularZRms,
            )
        val targetRpm =
            ticksPerSecondToWheelRpm(targetTicksPerSecond)
        val settled = isSettled(
            targetRpm = targetRpm,
            leftRpm = leftRpm,
            rightRpm = rightRpm,
        )
        detectEvents(
            targetRpm = targetRpm,
            leftRpm = leftRpm,
            rightRpm = rightRpm,
            settled = settled,
        )

        val events = ExperimentLogger.events(*pendingEvents.toTypedArray())
        pendingEvents.clear()

        val wroteRow = logger?.write(
            sample = ExperimentSample(
                sampleIndex = sample,
                timeS = timeSeconds,
                loopDtMs = loopDtMilliseconds,
                event = events,
                batteryV = voltageSensor.voltage,
            ),
            scenarioValues = mapOf(
                "scenario_direction" to directionName(),
                "scenario_drive_command" to normalizedCommand(),
                "scenario_gear_reduction" to DrivetrainConfig.GEAR_REDUCTION,
                "scenario_left_target_tps" to targetTicksPerSecond,
                "scenario_right_target_tps" to targetTicksPerSecond,
                "scenario_left_actual_tps" to leftVelocity,
                "scenario_right_actual_tps" to rightVelocity,
                "scenario_left_position_ticks" to leftMotor.currentPosition,
                "scenario_right_position_ticks" to rightMotor.currentPosition,
                "scenario_left_current_a" to leftCurrent,
                "scenario_right_current_a" to rightCurrent,
                "scenario_heading_deg" to
                    imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES),
                "scenario_yaw_rate_deg_s" to
                    angularVelocity.zRotationRate.toDouble(),
                "scenario_target_rpm" to targetRpm,
                "scenario_settled" to settled,
                "scenario_rpm_difference" to abs(leftRpm - rightRpm),
                "scenario_current_difference_a" to abs(leftCurrent - rightCurrent),
                "scenario_angular_vibration_rms_deg_s" to angularVibrationRms,
                "scenario_left_rpm" to leftRpm,
                "scenario_right_rpm" to rightRpm,
                "scenario_left_rpm_ripple_rms" to leftRpmDeviation.rms(),
                "scenario_right_rpm_ripple_rms" to rightRpmDeviation.rms(),
            ),
        ) ?: false
        sample += 1

        if (!wroteRow && phase !in TERMINAL_PHASES) {
            loggerError = logger?.errorMessage ?: "Logger unavailable"
            abort(ExperimentEvent.FAULT)
        }

        showTelemetry(
            leftRpm = leftRpm,
            rightRpm = rightRpm,
            leftCurrent = leftCurrent,
            rightCurrent = rightCurrent,
            angularVibrationRms = angularVibrationRms,
            settled = settled,
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

    private fun advanceProfile(now: Long) {
        val phaseSeconds = secondsBetween(phaseStartNanos, now)
        when (phase) {
            Phase.IDLE_FORWARD ->
                if (phaseSeconds >= IDLE_DURATION_SECONDS) {
                    transitionTo(
                        Phase.DRIVE_FORWARD,
                        ExperimentEvent.COMMAND_START,
                        now,
                    )
                }

            Phase.DRIVE_FORWARD ->
                if (phaseSeconds >= DRIVE_DURATION_SECONDS) {
                    transitionTo(
                        Phase.COAST_FORWARD,
                        ExperimentEvent.COMMAND_STOP,
                        now,
                    )
                }

            Phase.COAST_FORWARD ->
                if (phaseSeconds >= COAST_DURATION_SECONDS) {
                    val next = if (TEST_REVERSE) {
                        Phase.IDLE_REVERSE
                    } else {
                        Phase.COMPLETE
                    }
                    val event = if (TEST_REVERSE) {
                        ExperimentEvent.MARK
                    } else {
                        ExperimentEvent.END
                    }
                    transitionTo(next, event, now)
                }

            Phase.IDLE_REVERSE ->
                if (phaseSeconds >= IDLE_DURATION_SECONDS) {
                    transitionTo(
                        Phase.DRIVE_REVERSE,
                        ExperimentEvent.COMMAND_START,
                        now,
                    )
                }

            Phase.DRIVE_REVERSE ->
                if (phaseSeconds >= DRIVE_DURATION_SECONDS) {
                    transitionTo(
                        Phase.COAST_REVERSE,
                        ExperimentEvent.COMMAND_STOP,
                        now,
                    )
                }

            Phase.COAST_REVERSE ->
                if (phaseSeconds >= COAST_DURATION_SECONDS) {
                    transitionTo(
                        Phase.COMPLETE,
                        ExperimentEvent.END,
                        now,
                    )
                }

            else -> Unit
        }
    }

    private fun transitionTo(
        nextPhase: Phase,
        event: ExperimentEvent,
        now: Long,
    ) {
        phase = nextPhase
        phaseStartNanos = now
        pendingEvents += event
        resetPhaseMeasurements()

        when (nextPhase) {
            Phase.COAST_FORWARD,
            Phase.COAST_REVERSE,
            Phase.COMPLETE,
            Phase.ABORTED,
            -> setCoastMode()

            else -> setVelocityMode()
        }
    }

    private fun applyMotorCommand() {
        val direction = when (phase) {
            Phase.DRIVE_FORWARD -> 1.0
            Phase.DRIVE_REVERSE -> -1.0
            else -> 0.0
        }
        val maximumVelocity = minOf(
            leftMotor.motorType.achieveableMaxTicksPerSecond,
            rightMotor.motorType.achieveableMaxTicksPerSecond,
        )
        targetTicksPerSecond =
            direction * STANDARD_COMMAND * maximumVelocity

        if (
            phase == Phase.DRIVE_FORWARD ||
            phase == Phase.DRIVE_REVERSE
        ) {
            leftMotor.velocity = targetTicksPerSecond
            rightMotor.velocity = targetTicksPerSecond
        } else {
            when (phase) {
                Phase.WAITING,
                Phase.IDLE_FORWARD,
                Phase.IDLE_REVERSE,
                -> {
                    leftMotor.velocity = 0.0
                    rightMotor.velocity = 0.0
                }

                else -> stopMotors()
            }
        }
    }

    private fun setVelocityMode() {
        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private fun setCoastMode() {
        leftMotor.power = 0.0
        rightMotor.power = 0.0
        leftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        targetTicksPerSecond = 0.0
    }

    private fun stopMotors() {
        leftMotor.power = 0.0
        rightMotor.power = 0.0
    }

    private fun abort(event: ExperimentEvent) {
        if (phase == Phase.ABORTED) return
        pendingEvents += event
        phase = Phase.ABORTED
        stopMotors()
    }

    private fun detectEvents(
        targetRpm: Double,
        leftRpm: Double,
        rightRpm: Double,
        settled: Boolean,
    ) {
        if (
            phase == Phase.DRIVE_FORWARD ||
            phase == Phase.DRIVE_REVERSE
        ) {
            if (
                !motionDetected &&
                max(abs(leftRpm), abs(rightRpm)) >=
                abs(targetRpm) * MOTION_THRESHOLD_RATIO
            ) {
                motionDetected = true
                pendingEvents += ExperimentEvent.MARK
            }
            if (!settledDetected && settled) {
                settledDetected = true
                pendingEvents += ExperimentEvent.TARGET_REACHED
            }
        }

        if (
            (phase == Phase.COAST_FORWARD ||
                phase == Phase.COAST_REVERSE) &&
            !coastStoppedDetected &&
            max(abs(leftRpm), abs(rightRpm)) <= COAST_STOP_RPM
        ) {
            coastStoppedDetected = true
            pendingEvents += ExperimentEvent.STOPPED
        }
    }

    private fun isSettled(
        targetRpm: Double,
        leftRpm: Double,
        rightRpm: Double,
    ): Boolean {
        if (
            phase != Phase.DRIVE_FORWARD &&
            phase != Phase.DRIVE_REVERSE
        ) {
            return false
        }
        val tolerance = max(
            MIN_SETTLED_TOLERANCE_RPM,
            abs(targetRpm) * SETTLED_TOLERANCE_RATIO,
        )
        return abs(leftRpm - targetRpm) <= tolerance &&
            abs(rightRpm - targetRpm) <= tolerance
    }

    private fun updateRollingMeasurements(
        leftRpm: Double,
        rightRpm: Double,
        angularX: Double,
        angularY: Double,
        angularZ: Double,
    ) {
        leftRpmDeviation.add(leftRpm)
        rightRpmDeviation.add(rightRpm)
        angularXDeviation.add(angularX)
        angularYDeviation.add(angularY)
        angularZDeviation.add(angularZ)
    }

    private fun resetPhaseMeasurements() {
        leftRpmDeviation.reset()
        rightRpmDeviation.reset()
        angularXDeviation.reset()
        angularYDeviation.reset()
        angularZDeviation.reset()
        motionDetected = false
        settledDetected = false
        coastStoppedDetected = false
    }

    private fun normalizedCommand(): Double = when (phase) {
        Phase.DRIVE_FORWARD -> STANDARD_COMMAND
        Phase.DRIVE_REVERSE -> -STANDARD_COMMAND
        else -> 0.0
    }

    private fun directionName(): String = when (phase) {
        Phase.IDLE_FORWARD,
        Phase.DRIVE_FORWARD,
        Phase.COAST_FORWARD,
        -> "FORWARD"

        Phase.IDLE_REVERSE,
        Phase.DRIVE_REVERSE,
        Phase.COAST_REVERSE,
        -> "REVERSE"

        else -> "NONE"
    }

    private fun ticksPerSecondToWheelRpm(ticksPerSecond: Double): Double =
        ticksPerSecond * 60.0 /
            DrivetrainConfig.encoderTicksPerWheelRevolution

    private fun secondsBetween(startNanos: Long, endNanos: Long): Double =
        (endNanos - startNanos) / NANOS_PER_SECOND

    private fun showTelemetry(
        leftRpm: Double,
        rightRpm: Double,
        leftCurrent: Double,
        rightCurrent: Double,
        angularVibrationRms: Double,
        settled: Boolean,
    ) {
        telemetry.addData("DT00", phase.name)
        telemetry.addData(
            "Command",
            "%.0f%% | target=%.1f rpm",
            normalizedCommand() * 100.0,
            ticksPerSecondToWheelRpm(targetTicksPerSecond),
        )
        telemetry.addData(
            "RPM",
            "L=%.1f R=%.1f difference=%.1f settled=%s",
            leftRpm,
            rightRpm,
            abs(leftRpm - rightRpm),
            if (settled) "YES" else "NO",
        )
        telemetry.addData(
            "Current",
            "L=%.2f A R=%.2f A difference=%.2f A",
            leftCurrent,
            rightCurrent,
            abs(leftCurrent - rightCurrent),
        )
        telemetry.addData(
            "Vibration",
            "angular=%.3f deg/s RPM-RMS L=%.2f R=%.2f",
            angularVibrationRms,
            leftRpmDeviation.rms(),
            rightRpmDeviation.rms(),
        )
        telemetry.addData(
            "Log",
            "%s | rows=%d%s",
            logger?.fileName ?: logFileName ?: "unavailable",
            logger?.rowCount ?: finalRowCount,
            loggerError?.let { " | ERROR: $it" }.orEmpty(),
        )
        if (phase == Phase.WAITING) {
            telemetry.addData("Safety", "Press A only after robot is secured")
        }
    }

    private fun closeLogger() {
        finalRowCount = logger?.rowCount ?: finalRowCount
        logger?.close()
        loggerError = loggerError ?: logger?.errorMessage
        logger = null
    }

    private fun writeTerminalEvent(event: ExperimentEvent) {
        logger?.write(
            sample = ExperimentSample(
                sampleIndex = sample,
                timeS = if (runStartNanos == 0L) {
                    0.0
                } else {
                    secondsBetween(runStartNanos, System.nanoTime())
                },
                event = ExperimentLogger.events(event),
                batteryV = voltageSensor.voltage,
            ),
            scenarioValues = SCENARIO_COLUMNS.associateWith { null },
        )
        sample += 1
    }

    private fun driveMotor(
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
        const val LEFT_MOTOR_NAME = "left_drive"
        const val RIGHT_MOTOR_NAME = "right_drive"
        const val IMU_NAME = "imu"
        const val SCENARIO_ID = "DT00"
        const val CONFIGURATION_ID = "DT00_BASELINE"

        const val STANDARD_COMMAND = 0.50
        const val TEST_REVERSE = true
        const val IDLE_DURATION_SECONDS = 1.0
        const val DRIVE_DURATION_SECONDS = 4.0
        const val COAST_DURATION_SECONDS = 3.0

        const val RMS_WINDOW_SAMPLES = 25
        const val MOTION_THRESHOLD_RATIO = 0.05
        const val SETTLED_TOLERANCE_RATIO = 0.05
        const val MIN_SETTLED_TOLERANCE_RPM = 5.0
        const val COAST_STOP_RPM = 3.0

        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val NANOS_PER_MILLISECOND = 1_000_000.0

        val TERMINAL_PHASES = setOf(Phase.COMPLETE, Phase.ABORTED)

        val SCENARIO_COLUMNS = listOf(
            "scenario_direction",
            "scenario_drive_command",
            "scenario_gear_reduction",
            "scenario_left_target_tps",
            "scenario_right_target_tps",
            "scenario_left_actual_tps",
            "scenario_right_actual_tps",
            "scenario_left_position_ticks",
            "scenario_right_position_ticks",
            "scenario_left_current_a",
            "scenario_right_current_a",
            "scenario_heading_deg",
            "scenario_yaw_rate_deg_s",
            "scenario_target_rpm",
            "scenario_settled",
            "scenario_rpm_difference",
            "scenario_current_difference_a",
            "scenario_angular_vibration_rms_deg_s",
            "scenario_left_rpm",
            "scenario_right_rpm",
            "scenario_left_rpm_ripple_rms",
            "scenario_right_rpm_ripple_rms",
        )
    }
}
