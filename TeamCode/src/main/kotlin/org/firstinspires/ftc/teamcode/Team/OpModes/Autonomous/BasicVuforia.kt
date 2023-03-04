package org.firstinspires.ftc.teamcode.Team.OpModes.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



/**
 * This OpMode illustrates using the Vuforia localizer to determine positioning and orientation of
 * robot on the FTC field using the RC phone's camera.  The code is structured as a LinearOpMode
 *
 * Note: If you are using a WEBCAM see ConceptVuforiaFieldNavigationWebcam.java
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * Finally, the location of the camera on the robot is used to determine the
 * robot's location and orientation on the field.
 *
 * To learn more about the FTC field coordinate model, see FTC_FieldCoordinateSystemDefinition.pdf in this folder
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Vuforia Field Nav", group = "Concept")
@Disabled
class BasicVuforia : LinearOpMode() {
    // Class Members
    private var lastLocation: OpenGLMatrix? = null
    private var vuforia: VuforiaLocalizer? = null
    private var targets: VuforiaTrackables? = null
    private var targetVisible = false
    private var phoneXRotate = 0f
    private var phoneYRotate = 0f
    private val phoneZRotate = 0f
    override fun runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = CAMERA_CHOICE

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Load the trackable assets.
        targets = vuforia!!.loadTrackablesFromAsset("PowerPlay")

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        val allTrackables: MutableList<VuforiaTrackable> = ArrayList()
        allTrackables.addAll(targets!!)
        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of *transformation matrices.*
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See [Transformation Matrix](https://en.wikipedia.org/wiki/Transformation_matrix)
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the [OpenGLMatrix] class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         * - The X axis runs from your left to the right. (positive from the center to the right)
         * - The Y axis runs from the Red Alliance Station towards the other side of the field
         * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(
            0,
            "Red Audience Wall",
            -halfField,
            -oneAndHalfTile,
            mmTargetHeight,
            90f,
            0f,
            90f
        )
        identifyTarget(
            1,
            "Red Rear Wall",
            halfField,
            -oneAndHalfTile,
            mmTargetHeight,
            90f,
            0f,
            -90f
        )
        identifyTarget(
            2,
            "Blue Audience Wall",
            -halfField,
            oneAndHalfTile,
            mmTargetHeight,
            90f,
            0f,
            90f
        )
        identifyTarget(
            3,
            "Blue Rear Wall",
            halfField,
            oneAndHalfTile,
            mmTargetHeight,
            90f,
            0f,
            -90f
        )

        /*
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
         * Lock it into Portrait for these numbers to work.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a heading angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.
         * The two examples below assume that the camera is facing forward out the front of the robot.
         */

        // We need to rotate the camera around its long axis to bring the correct camera forward.
        phoneYRotate = if (CAMERA_CHOICE == CameraDirection.BACK) {
            -90f
        } else {
            90f
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90f
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
        val CAMERA_FORWARD_DISPLACEMENT =
            0.0f * mmPerInch // eg: Enter the forward distance from the center of the robot to the camera lens
        val CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch // eg: Camera is 6 Inches above ground
        val CAMERA_LEFT_DISPLACEMENT =
            0.0f * mmPerInch // eg: Enter the left distance from the center of the robot to the camera lens
        val robotFromCamera = OpenGLMatrix
            .translation(
                CAMERA_FORWARD_DISPLACEMENT,
                CAMERA_LEFT_DISPLACEMENT,
                CAMERA_VERTICAL_DISPLACEMENT
            )
            .multiplied(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.YZX,
                    AngleUnit.DEGREES,
                    phoneYRotate,
                    phoneZRotate,
                    phoneXRotate
                )
            )
        /**  Let all the trackable listeners know where the phone is.   */
        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setPhoneInformation(
                robotFromCamera,
                parameters.cameraDirection
            )
        }

        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Stream feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */

        // waitForStart();

        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */
        targets!!.activate()
        while (!isStopRequested) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false
            for (trackable in allTrackables) {
                if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                    telemetry.addData("Visible Target", trackable.name)
                    targetVisible = true

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    val robotLocationTransform =
                        (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform
                    }
                    break
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                val translation = lastLocation!!.translation
                telemetry.addData(
                    "Pos (inches)",
                    "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation[0] / mmPerInch,
                    translation[1] / mmPerInch,
                    translation[2] / mmPerInch
                )

                // express the rotation of the robot in degrees.
                val rotation = Orientation.getOrientation(
                    lastLocation,
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
                )
                telemetry.addData(
                    "Rot (deg)",
                    "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                    rotation.firstAngle,
                    rotation.secondAngle,
                    rotation.thirdAngle
                )
            } else {
                telemetry.addData("Visible Target", "none")
            }
            telemetry.update()
        }

        // Disable Tracking when we are done;
        targets!!.deactivate()
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    fun identifyTarget(
        targetIndex: Int,
        targetName: String?,
        dx: Float,
        dy: Float,
        dz: Float,
        rx: Float,
        ry: Float,
        rz: Float
    ) {
        val aTarget = targets!![targetIndex]
        aTarget.name = targetName
        aTarget.location = OpenGLMatrix.translation(dx, dy, dz)
            .multiplied(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES,
                    rx,
                    ry,
                    rz
                )
            )
    }

    companion object {
        // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
        // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
        // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
        private val CAMERA_CHOICE = CameraDirection.BACK
        private const val PHONE_IS_PORTRAIT = false

        /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
        private const val VUFORIA_KEY = " -- YOUR NEW VUFORIA KEY GOES HERE  --- "

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here.  These are useful for the FTC competition field.
        private const val mmPerInch = 25.4f
        private const val mmTargetHeight =
            6 * mmPerInch // the height of the center of the target image above the floor
        private const val halfField = 72 * mmPerInch
        private const val halfTile = 12 * mmPerInch
        private const val oneAndHalfTile = 36 * mmPerInch
    }
}
