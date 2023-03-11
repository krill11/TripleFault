package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
class StraightTest : LinearOpMode() {
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val telemetry: Telemetry =
            MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry())
        val drive = SampleMecanumDrive(hardwareMap)
        val trajectory: Trajectory = drive.trajectoryBuilder(Pose2d())
            .forward(DISTANCE)
            .build()
        waitForStart()
        if (isStopRequested()) return
        drive.followTrajectory(trajectory)
        val poseEstimate: Pose2d = drive.getPoseEstimate()
        telemetry.addData("finalX", poseEstimate.getX())
        telemetry.addData("finalY", poseEstimate.getY())
        telemetry.addData("finalHeading", poseEstimate.getHeading())
        telemetry.update()
        while (!isStopRequested() && opModeIsActive());
    }

    companion object {
        var DISTANCE = 60.0 // in
    }
}