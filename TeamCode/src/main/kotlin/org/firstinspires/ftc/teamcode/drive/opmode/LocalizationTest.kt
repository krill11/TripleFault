package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
class LocalizationTest : LinearOpMode() {
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
                )
            )
            drive.update()
            val poseEstimate: Pose2d = drive.getPoseEstimate()
            telemetry.addData("x", poseEstimate.getX())
            telemetry.addData("y", poseEstimate.getY())
            telemetry.addData("heading", poseEstimate.getHeading())
            telemetry.update()
        }
    }
}