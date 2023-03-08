package org.firstinspires.ftc.teamcode.Team.ComplexRobots

import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Team.BasicRobots.Mecanum
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.*

/**
 *    This file is here for the addition of other hardware components onto the basic robot
 *    and for the definition of functions to be later used in the manual control modes
 */
class Robot : Mecanum() {
    var translation = VectorF(0.0F, 0.0F, 0.0F)
    /**
     *   firstAngle  is  roll
     *   secondAngle is  pitch
     *   thirdAngle  is  yaw   (heading)
     */
    var orientation = Orientation()
    private var lastLocation: OpenGLMatrix = OpenGLMatrix()
    private var vuforia: VuforiaLocalizer? = null
    private var targets: VuforiaTrackables? = null
    private var webcamName: WebcamName? = null
    private var targetVisible = false
    private var allTrackables: MutableList<VuforiaTrackable> = ArrayList()

    override fun init(hardwareMap: HardwareMap) {
        super.init(hardwareMap)

        initVuforia(hardwareMap);
    }

    /**
     *   updatePosition will return true if the position was updated, and false otherwise
     */
    fun updatePosition(): Boolean {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false
        for (trackable in allTrackables) {
            if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                targetVisible = true

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                val transform =
                    (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                if (transform != null) {
                    lastLocation = transform
                    translation = transform.translation
                    orientation = Orientation.getOrientation(
                        lastLocation,
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES
                    )
                }
                return true;
            }
        }
        return false;
    }

    private fun initVuforia(hardwareMap: HardwareMap) {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName::class.java, "FrontCam")

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )

        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = true

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = vuforia!!.loadTrackablesFromAsset("PowerPlay")

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
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
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */
        val CAMERA_FORWARD_DISPLACEMENT =
            3.125f * mmPerInch // eg: Enter the forward distance from the center of the robot to the camera lens
        val CAMERA_VERTICAL_DISPLACEMENT = 11.0f * mmPerInch // eg: Camera is 6 Inches above ground
        val CAMERA_LEFT_DISPLACEMENT =
            0.0f * mmPerInch // eg: Enter the left distance from the center of the robot to the camera lens
        val cameraLocationOnRobot = OpenGLMatrix
            .translation(
                CAMERA_FORWARD_DISPLACEMENT,
                CAMERA_LEFT_DISPLACEMENT,
                CAMERA_VERTICAL_DISPLACEMENT
            )
            .multiplied(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC,
                    AxesOrder.XZY,
                    AngleUnit.DEGREES,
                    90f,
                    90f,
                    0f
                )
            )
        /**  Let all the trackable listeners know where the camera is.   */
        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setCameraLocationOnRobot(
                parameters.cameraName!!, cameraLocationOnRobot
            )
        }

        targets!!.activate()
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    private fun identifyTarget(
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
        private val VUFORIA_KEY = System.getProperty("VUFORIA_KEY") ?: throw Exception("VUFORIA_KEY is missing");

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private const val mmPerInch = 25.4f
        private const val mmTargetHeight =
            6 * mmPerInch // the height of the center of the target image above the floor
        private const val halfField = 72 * mmPerInch
        private const val halfTile = 12 * mmPerInch
        private const val oneAndHalfTile = 36 * mmPerInch
    }
}