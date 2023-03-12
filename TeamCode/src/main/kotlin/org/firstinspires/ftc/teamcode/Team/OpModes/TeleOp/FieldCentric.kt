package org.firstinspires.ftc.teamcode.Team.OpModes.TeleOp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name="Field Centric Strafing")
class FieldCentric : LinearOpMode() {
    var robot = Robot()
    val mmPerInch = 25.4f
    @Throws(InterruptedException::class)

    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        robot.init(hardwareMap)

        waitForStart();

        val imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x * 1.1
            val rx = gamepad1.right_stick_x.toDouble()
            val botHeading = -imu.angularOrientation.firstAngle.toDouble()
            val rotX = x * cos(botHeading) - y * sin(botHeading)
            val rotY = x * sin(botHeading) + y * cos(botHeading)
            val denominator = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)

            val lfpower = (rotY + rotX + rx) / denominator
            val lbpower = (rotY - rotX + rx) / denominator
            val rfpower = (rotY - rotX - rx) / denominator
            val rbpower = (rotY + rotX - rx) / denominator

            robot.setPower(mapOf(
                "LFMotor" to lfpower,
                "LBMotor" to lbpower,
                "RFMotor" to rfpower,
                "RBMotor" to rbpower,
            ))

            val packet = TelemetryPacket()
            robot.updatePosition()
            val translation = robot.translation
            val rotation = robot.orientation
            var radianHeading = Math.toRadians((rotation.thirdAngle).toString().toDouble()) + (Math.PI/4)
            var RealRadianHeading = Math.toRadians((rotation.thirdAngle).toString().toDouble())
            val xpoints: DoubleArray = doubleArrayOf((translation[0]/ mmPerInch)+(cos(radianHeading))*9, (translation[0]/ mmPerInch)+(sin(radianHeading))*9, (translation[0]/ mmPerInch)-(cos(radianHeading))*9, (translation[0]/ mmPerInch)-(sin(radianHeading))*9, (translation[0]/ mmPerInch)+(cos(radianHeading))*9)
            val ypoints: DoubleArray = doubleArrayOf((translation[1]/ mmPerInch)+(sin(radianHeading))*9, (translation[1]/ mmPerInch)-(cos(radianHeading))*9, (translation[1]/ mmPerInch)-(sin(radianHeading))*9, (translation[1]/ mmPerInch)+(cos(radianHeading))*9, (translation[1]/ mmPerInch)+(sin(radianHeading))*9)
            val xface: DoubleArray = doubleArrayOf(((translation[0].toString().toDouble())/ mmPerInch), (translation[0]/ mmPerInch)+(cos(RealRadianHeading))*9)
            val yface: DoubleArray = doubleArrayOf(((translation[1].toString().toDouble())/ mmPerInch), (translation[1]/ mmPerInch)+(sin(RealRadianHeading))*9)
            packet.put("x", translation[0] / mmPerInch)
            packet.put("y", translation[1] / mmPerInch)
            packet.fieldOverlay()
                .setFill("blue")
                .setStroke("black")
                .strokePolyline(xpoints, ypoints)
                .setStroke("red")
                .strokePolyline(xface, yface)
            dashboard.sendTelemetryPacket(packet)
        }
    }
}