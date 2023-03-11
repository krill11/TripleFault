package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
class SplineTest : LinearOpMode() {
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        waitForStart()
        if (isStopRequested()) return
        val traj: Trajectory = drive.trajectoryBuilder(Pose2d())
            .splineTo(Vector2d(30, 30), 0)
            .build()
        drive.followTrajectory(traj)
        sleep(2000)
        drive.followTrajectory(
            drive.trajectoryBuilder(traj.end(), true)
                .splineTo(Vector2d(0, 0), Math.toRadians(180))
                .build()
        )
    }
}