package fgc.vietnam.robot01

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

enum class DriveControlMode {
    ARCADE,
    TANK,
}

abstract class SharedDriveTeleOp protected constructor(
    private val controlMode: DriveControlMode,
) : OpMode() {
    private lateinit var drivetrain: Drivetrain
    private lateinit var flywheel: Flywheel

    private var previousHeadingToggle = false
    private var previousFlywheelToggle = false
    private var previousFlywheelIncrease = false
    private var previousFlywheelDecrease = false

    override fun init() {
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        telemetry.addData(
            "Status",
            if (controlMode == DriveControlMode.ARCADE) {
                "A heading hold | B flywheel | D-pad Up/Down RPM"
            } else {
                "Tank: sticks Y | B flywheel | D-pad Up/Down RPM"
            },
        )
    }

    override fun loop() {
        val headingToggle = gamepad1.a
        if (
            controlMode == DriveControlMode.ARCADE &&
            headingToggle &&
            !previousHeadingToggle
        ) {
            drivetrain.toggleHeadingHold()
        }
        previousHeadingToggle = headingToggle

        val flywheelToggle = gamepad1.b
        if (flywheelToggle && !previousFlywheelToggle) {
            flywheel.toggle()
        }
        previousFlywheelToggle = flywheelToggle

        val flywheelIncrease = gamepad1.dpad_up
        if (flywheelIncrease && !previousFlywheelIncrease) {
            flywheel.increaseSpeed()
        }
        previousFlywheelIncrease = flywheelIncrease

        val flywheelDecrease = gamepad1.dpad_down
        if (flywheelDecrease && !previousFlywheelDecrease) {
            flywheel.decreaseSpeed()
        }
        previousFlywheelDecrease = flywheelDecrease

        val leftStickY =
            deadband(-gamepad1.left_stick_y.toDouble())
        val rightStickY =
            deadband(-gamepad1.right_stick_y.toDouble())
        val rightStickX =
            deadband(-gamepad1.right_stick_x.toDouble())
        val drive = when (controlMode) {
            DriveControlMode.ARCADE -> drivetrain.drive(
                forward = leftStickY,
                turn = rightStickX,
            )

            DriveControlMode.TANK -> drivetrain.driveTank(
                left = rightStickY,
                right = leftStickY,
            )
        }
        val flywheelState = flywheel.update()

        when (controlMode) {
            DriveControlMode.ARCADE -> telemetry.addData(
                "Input",
                "forward=%.2f→%.2f turn=%.2f hold=%s",
                drive.requestedForward,
                drive.limitedForward,
                drive.requestedTurn,
                if (drive.headingHoldEnabled) "ON" else "OFF",
            )

            DriveControlMode.TANK -> telemetry.addData(
                "Input",
                "left motor(R stick)=%.2f right motor(L stick)=%.2f",
                rightStickY,
                leftStickY,
            )
        }
        telemetry.addData(
            "Heading",
            "now=%.1f° target=%.1f° error=%+.2f° rate=%+.1f°/s",
            drive.heading,
            drive.targetHeading,
            drive.headingError,
            drive.yawRate,
        )
        telemetry.addData(
            "Heading PID",
            "P=%+.3f I=%+.3f D=%+.3f total=%+.3f",
            drive.proportionalCorrection,
            drive.integralCorrection,
            drive.derivativeCorrection,
            drive.headingCorrection,
        )
        telemetry.addData(
            "Velocity",
            "L=%.0f/%.0f R=%.0f/%.0f ticks/s",
            drive.leftActualVelocity,
            drive.leftTargetVelocity,
            drive.rightActualVelocity,
            drive.rightTargetVelocity,
        )
        telemetry.addData(
            "Drive current",
            "L=%.2f A R=%.2f A total=%.2f A",
            drive.leftCurrentAmps,
            drive.rightCurrentAmps,
            drive.leftCurrentAmps + drive.rightCurrentAmps,
        )
        telemetry.addData(
            "Robot",
            "speed=%.0f mm/s battery=%.2f V gear=%.1f:1",
            drive.linearSpeedMmPerSecond,
            drive.batteryVoltage,
            DrivetrainConfig.GEAR_REDUCTION,
        )
        telemetry.addData(
            "Flywheel",
            "%s %s shaft=%.0f/%.0f rpm difference=%.0f",
            if (flywheelState.enabled) "ON" else "OFF",
            if (flywheelState.atSpeed) "READY" else "—",
            flywheelState.shaftRpm,
            flywheelState.targetRpm,
            flywheelState.rpmDifference,
        )
        telemetry.addData(
            "Flywheel M2",
            "rpm=%.0f velocity=%.0f/%.0f ticks/s current=%.2f A",
            flywheelState.primaryMotor.rpm,
            flywheelState.primaryMotor.velocity,
            flywheelState.targetVelocity,
            flywheelState.primaryMotor.currentAmps,
        )
        telemetry.addData(
            "Flywheel M3",
            "rpm=%.0f velocity=%.0f/%.0f ticks/s current=%.2f A",
            flywheelState.secondaryMotor.rpm,
            flywheelState.secondaryMotor.velocity,
            flywheelState.targetVelocity,
            flywheelState.secondaryMotor.currentAmps,
        )
        telemetry.addData(
            "Flywheel wheel",
            "rim=%.1f m/s",
            flywheelState.surfaceSpeedMetersPerSecond,
        )
    }

    override fun stop() {
        drivetrain.stop()
        flywheel.stop()
    }

    private fun deadband(value: Double): Double =
        if (abs(value) < JOYSTICK_DEADBAND) 0.0 else value

    private companion object {
        const val JOYSTICK_DEADBAND = 0.05
    }
}

@TeleOp(name = "FGC: Arcade Drive", group = "FGC Vietnam")
class DrivetrainTeleOp :
    SharedDriveTeleOp(DriveControlMode.ARCADE)
