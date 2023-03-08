package org.firstinspires.ftc.teamcode.Team.OpModes.Autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import kotlin.math.cos
import kotlin.math.sin


@TeleOp(name = "Field Centric Strafe (DashBoard)")
class FieldCentricDash : LinearOpMode() {
    private var lastLocation: OpenGLMatrix? = null
    private var vuforia: VuforiaLocalizer? = null
    private var targets: VuforiaTrackables? = null
    private var webcamName: WebcamName? = null
    private var targetVisible = false

    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()

        waitForStart();

        webcamName = hardwareMap.get(WebcamName::class.java, "FrontCam")
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraName = webcamName
        parameters.useExtendedTracking = true
        vuforia = ClassFactory.getInstance().createVuforia(parameters)
        targets = vuforia!!.loadTrackablesFromAsset("PowerPlay")
        val allTrackables: MutableList<VuforiaTrackable> = ArrayList()
        allTrackables.addAll(targets!!)
        identifyTarget(0, "Red Audience Wall", -halfField, -oneAndHalfTile, mmTargetHeight, 90f, 0f, 90f)
        identifyTarget(1, "Red Rear Wall", halfField, -oneAndHalfTile, mmTargetHeight, 90f, 0f, -90f)
        identifyTarget(2, "Blue Audience Wall", -halfField, oneAndHalfTile, mmTargetHeight, 90f, 0f, 90f)
        identifyTarget(3, "Blue Rear Wall", halfField, oneAndHalfTile, mmTargetHeight, 90f, 0f, -90f)
        val CAMERA_FORWARD_DISPLACEMENT =
            3.125f * mmPerInch // eg: Enter the forward distance from the center of the robot to the camera lens
        val CAMERA_VERTICAL_DISPLACEMENT = 11.0f * mmPerInch // eg: Camera is 6 Inches above ground
        val CAMERA_LEFT_DISPLACEMENT =
            0.0f * mmPerInch // eg: Enter the left distance from the center of the robot to the camera lens
        val cameraLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT
            )
            .multiplied(
                Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES, 90f, 90f, 0f
                )
            )
        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setCameraLocationOnRobot(
                parameters.cameraName!!, cameraLocationOnRobot
            )
        }
        targets!!.activate()
        while (!isStopRequested) {
            targetVisible = false
            for (trackable in allTrackables) {
                if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                    telemetry.addData("Visible Target", trackable.name)
                    targetVisible = true
                    val robotLocationTransform =
                        (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform
                    }
                    break
                }
            }
            if (targetVisible) {
                val translation = lastLocation!!.translation
                val packet = TelemetryPacket()
                packet.put("status", "alive")
                packet.put("x", 0)
                packet.put("y", 0)

                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f", translation[0] / mmPerInch, translation[1] / mmPerInch, translation[2] / mmPerInch
                )
                val rotation = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES
                )
                var RealRadianHeading = Math.toRadians((rotation.thirdAngle).toString().toDouble())
                var radianHeading = Math.toRadians((rotation.thirdAngle).toString().toDouble()) + (Math.PI/4)
                val xpoints: DoubleArray = doubleArrayOf((translation[0]/ mmPerInch)+(cos(radianHeading))*9, (translation[0]/ mmPerInch)+(sin(radianHeading))*9, (translation[0]/ mmPerInch)-(cos(radianHeading))*9, (translation[0]/ mmPerInch)-(sin(radianHeading))*9, (translation[0]/ mmPerInch)+(cos(radianHeading))*9)
                val ypoints: DoubleArray = doubleArrayOf((translation[1]/ mmPerInch)+(sin(radianHeading))*9, (translation[1]/ mmPerInch)-(cos(radianHeading))*9, (translation[1]/ mmPerInch)-(sin(radianHeading))*9, (translation[1]/ mmPerInch)+(cos(radianHeading))*9, (translation[1]/ mmPerInch)+(sin(radianHeading))*9)
                val xface: DoubleArray = doubleArrayOf(((translation[0].toString().toDouble())/ mmPerInch), (translation[0]/ mmPerInch)+(cos(RealRadianHeading))*9)
                val yface: DoubleArray = doubleArrayOf(((translation[1].toString().toDouble())/ mmPerInch), (translation[1]/ mmPerInch)+(sin(RealRadianHeading))*9)
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle,rotation.secondAngle, rotation.thirdAngle
                )
                packet.put("x", translation[0] / mmPerInch)
                packet.put("y", translation[1] / mmPerInch)
                packet.fieldOverlay()
                    .setFill("blue")
                    .setStroke("black")
                    .strokePolyline(xpoints, ypoints)
                dashboard.sendTelemetryPacket(packet)
                telemetry.addData("X1", xpoints[0])
                telemetry.addData("Y1", ypoints[0])
                telemetry.addData("X2", xpoints[1])
                telemetry.addData("Y2", ypoints[1])
                telemetry.addData("Heading (radians)", radianHeading)
            } else {
                telemetry.addData("Visible Target", "none")
            }
            telemetry.update()
        }
        targets!!.deactivate()
    }
    fun identifyTarget(
        targetIndex: Int, targetName: String?, dx: Float, dy: Float, dz: Float, rx: Float, ry: Float,rz: Float
    ) {
        val aTarget = targets!![targetIndex]
        aTarget.name = targetName
        aTarget.location = OpenGLMatrix.translation(dx, dy, dz)
            .multiplied(
                Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, rx, ry, rz)
            )
    }
    companion object {
        private const val VUFORIA_KEY = "ATuhS5P/////AAABmR8UXvLT90ReroEHDfFojsI7Aa6SANQWVEBufLRaRQVNvLfnvDga0q1F7LQ3sFTKew0TFr6fHjRnvZxKz6g03jljG3Ou26MSob0I59jRyD6NTV6NP0V87CnZ+BMvs8vvAW1lIYJISJBR9wjnhQBk9NnF73J2AXo29TrS7Z+OdmYNsom0JTg6FaIx/aRkhOXVaiVOEs6yC3yRZqLsN2GRJYu3YDxw6O5JYbyIVRnjpRTq5A8HmVKcZsWfkJASiLxpGVP+KWewfd6q/4SgdPvHzrp6yCE4rbfBbhNuxU1xJZDp9rH3CcVPkU7sqmoZezogL0Le60wzRRgQR5GUJ+dj5BCzXIwI1MXPQIgdwdRbTgdm"
        private const val mmPerInch = 25.4f
        private const val mmTargetHeight =
            6 * mmPerInch
        private const val halfField = 72 * mmPerInch
        private const val halfTile = 12 * mmPerInch
        private const val oneAndHalfTile = 36 * mmPerInch
    }
}