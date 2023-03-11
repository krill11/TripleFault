package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.config.Config

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
class TurnTest : LinearOpMode() {
    @Override
    @Throws(InterruptedException::class)
    fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        waitForStart()
        if (isStopRequested()) return
        drive.turn(Math.toRadians(ANGLE))
    }

    companion object {
        var ANGLE = 90.0 // deg
    }
}