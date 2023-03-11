package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 *
 *
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 *
 *
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */
@Config
@Autonomous(group = "drive")
class MaxAngularVeloTuner : LinearOpMode() {
    private var timer: ElapsedTime? = null
    private var maxAngVelocity = 0.0
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry())
        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.")
        telemetry.addLine("Please ensure you have enough space cleared.")
        telemetry.addLine("")
        telemetry.addLine("Press start when ready.")
        telemetry.update()
        waitForStart()
        telemetry.clearAll()
        telemetry.update()
        drive.setDrivePower(Pose2d(0, 0, 1))
        timer = ElapsedTime()
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate()
            val poseVelo: Pose2d = Objects.requireNonNull(
                drive.getPoseVelocity(),
                "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer."
            )
            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity)
        }
        drive.setDrivePower(Pose2d())
        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity)
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity))
        telemetry.addData("Max Recommended Angular Velocity (rad)", maxAngVelocity * 0.8)
        telemetry.addData(
            "Max Recommended Angular Velocity (deg)",
            Math.toDegrees(maxAngVelocity * 0.8)
        )
        telemetry.update()
        while (!isStopRequested()) idle()
    }

    companion object {
        var RUNTIME = 4.0
    }
}