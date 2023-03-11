package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder
import java.util.Arrays
import java.util.List

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class StandardTrackingWheelLocalizer(
    hardwareMap: HardwareMap,
    lastTrackingEncPositions: List<Integer>,
    lastTrackingEncVels: List<Integer>
) : ThreeTrackingWheelLocalizer(
    Arrays.asList(
        Pose2d(0, LATERAL_DISTANCE / 2, 0),  // left
        Pose2d(0, -LATERAL_DISTANCE / 2, 0),  // right
        Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
    )
) {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val frontEncoder: Encoder
    private val lastEncPositions: List<Integer>
    private val lastEncVels: List<Integer>

    init {
        lastEncPositions = lastTrackingEncPositions
        lastEncVels = lastTrackingEncVels
        leftEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "leftEncoder"))
        rightEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "rightEncoder"))
        frontEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "frontEncoder"))

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    @get:Override
    @get:NonNull
    val wheelPositions: List<Double>
        get() {
            val leftPos: Int = leftEncoder.getCurrentPosition()
            val rightPos: Int = rightEncoder.getCurrentPosition()
            val frontPos: Int = frontEncoder.getCurrentPosition()
            lastEncPositions.clear()
            lastEncPositions.add(leftPos)
            lastEncPositions.add(rightPos)
            lastEncPositions.add(frontPos)
            return Arrays.asList(
                encoderTicksToInches(leftPos.toDouble()),
                encoderTicksToInches(rightPos.toDouble()),
                encoderTicksToInches(frontPos.toDouble())
            )
        }

    @get:Override
    @get:NonNull
    val wheelVelocities: List<Double>
        get() {
            val leftVel = leftEncoder.getCorrectedVelocity() as Int
            val rightVel = rightEncoder.getCorrectedVelocity() as Int
            val frontVel = frontEncoder.getCorrectedVelocity() as Int
            lastEncVels.clear()
            lastEncVels.add(leftVel)
            lastEncVels.add(rightVel)
            lastEncVels.add(frontVel)
            return Arrays.asList(
                encoderTicksToInches(leftVel.toDouble()),
                encoderTicksToInches(rightVel.toDouble()),
                encoderTicksToInches(frontVel.toDouble())
            )
        }

    companion object {
        var TICKS_PER_REV = 0.0
        var WHEEL_RADIUS = 2.0 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        var LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels
        var FORWARD_OFFSET = 4.0 // in; offset of the lateral wheel
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}