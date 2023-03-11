package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard

/**
 * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
 * will also calculate the effective kF value for your velocity PID.
 *
 *
 * Upon pressing start, your bot will run at max power for RUNTIME seconds.
 *
 *
 * Further fine tuning of kF may be desired.
 */
@Config
@Autonomous(group = "drive")
class MaxVelocityTuner : LinearOpMode() {
    private var timer: ElapsedTime? = null
    private var maxVelocity = 0.0
    private var batteryVoltageSensor: VoltageSensor? = null
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry())
        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.")
        telemetry.addLine("Please ensure you have enough space cleared.")
        telemetry.addLine("")
        telemetry.addLine("Press start when ready.")
        telemetry.update()
        waitForStart()
        telemetry.clearAll()
        telemetry.update()
        drive.setDrivePower(Pose2d(1, 0, 0))
        timer = ElapsedTime()
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate()
            val poseVelo: Pose2d = Objects.requireNonNull(
                drive.getPoseVelocity(),
                "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer."
            )
            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity)
        }
        drive.setDrivePower(Pose2d())
        val effectiveKf: Double = DriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity))
        telemetry.addData("Max Velocity", maxVelocity)
        telemetry.addData("Max Recommended Velocity", maxVelocity * 0.8)
        telemetry.addData(
            "Voltage Compensated kF",
            effectiveKf * batteryVoltageSensor.getVoltage() / 12
        )
        telemetry.update()
        while (!isStopRequested() && opModeIsActive()) idle()
    }

    private fun veloInchesToTicks(inchesPerSec: Double): Double {
        return inchesPerSec / (2 * Math.PI * DriveConstants.WHEEL_RADIUS) / DriveConstants.GEAR_RATIO * DriveConstants.TICKS_PER_REV
    }

    companion object {
        var RUNTIME = 2.0
    }
}