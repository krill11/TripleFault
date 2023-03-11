package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import java.util.ArrayList
import java.util.Arrays
import java.util.List
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID
import org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER
import org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.drive.DriveConstants.kV

/*
 * Simple tank drive hardware implementation for REV hardware.
 */
@Config
class SampleTankDrive(hardwareMap: HardwareMap) : TankDrive(kV, kA, kStatic, TRACK_WIDTH) {
    private val trajectorySequenceRunner: TrajectorySequenceRunner
    private val follower: TrajectoryFollower
    private val motors: List<DcMotorEx>
    private val leftMotors: List<DcMotorEx>
    private val rightMotors: List<DcMotorEx>
    private val imu: IMU
    private val batteryVoltageSensor: VoltageSensor

    init {
        follower = TankPIDVAFollower(
            AXIAL_PID, CROSS_TRACK_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters: IMU.Parameters = Parameters(
            RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR
            )
        )
        imu.initialize(parameters)

        // add/remove motors depending on your robot (e.g., 6WD)
        val leftFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        val leftRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        val rightRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        val rightFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront)
        leftMotors = Arrays.asList(leftFront, leftRear)
        rightMotors = Arrays.asList(rightFront, rightRear)
        for (motor in motors) {
            val motorConfigurationType: MotorConfigurationType = motor.getMotorType().clone()
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0)
            motor.setMotorType(motorConfigurationType)
        }
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        trajectorySequenceRunner = TrajectorySequenceRunner(
            follower, HEADING_PID, batteryVoltageSensor,
            ArrayList(), ArrayList(), ArrayList(), ArrayList()
        )
    }

    fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, accelConstraint)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d?): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, accelConstraint,
            MAX_ANG_VEL, MAX_ANG_ACCEL
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(getPoseEstimate())
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence?) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence?) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    val lastError: Pose2d
        get() = trajectorySequenceRunner.getLastPoseError()

    fun update() {
        updatePoseEstimate()
        val signal: DriveSignal =
            trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity())
        if (signal != null) setDriveSignal(signal)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy) update()
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunner.isBusy()

    fun setMode(runMode: DcMotor.RunMode?) {
        for (motor in motors) {
            motor.setMode(runMode)
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior)
        }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel: Pose2d? = drivePower
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            val denom: Double = (VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading()))
            vel = Pose2d(
                VX_WEIGHT * drivePower.getX(),
                0,
                OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom)
        } else {
            // Ensure the y axis is zeroed out.
            vel = Pose2d(drivePower.getX(), 0, drivePower.getHeading())
        }
        setDrivePower(vel)
    }

    @get:Override
    @get:NonNull
    val wheelPositions: List<Double>
        get() {
            var leftSum = 0.0
            var rightSum = 0.0
            for (leftMotor in leftMotors) {
                leftSum += encoderTicksToInches(leftMotor.getCurrentPosition())
            }
            for (rightMotor in rightMotors) {
                rightSum += encoderTicksToInches(rightMotor.getCurrentPosition())
            }
            return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size())
        }
    val wheelVelocities: List<Double>
        get() {
            var leftSum = 0.0
            var rightSum = 0.0
            for (leftMotor in leftMotors) {
                leftSum += encoderTicksToInches(leftMotor.getVelocity())
            }
            for (rightMotor in rightMotors) {
                rightSum += encoderTicksToInches(rightMotor.getVelocity())
            }
            return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size())
        }

    @Override
    fun setMotorPowers(v: Double, v1: Double) {
        for (leftMotor in leftMotors) {
            leftMotor.setPower(v)
        }
        for (rightMotor in rightMotors) {
            rightMotor.setPower(v1)
        }
    }

    @get:Override
    val rawExternalHeading: Double
        get() = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)

    @get:Override
    val externalHeadingVelocity: Double
        get() = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate

    companion object {
        var AXIAL_PID: PIDCoefficients = PIDCoefficients(0, 0, 0)
        var CROSS_TRACK_PID: PIDCoefficients = PIDCoefficients(0, 0, 0)
        var HEADING_PID: PIDCoefficients = PIDCoefficients(0, 0, 0)
        var VX_WEIGHT = 1.0
        var OMEGA_WEIGHT = 1.0
        private val VEL_CONSTRAINT: TrajectoryVelocityConstraint =
            getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH)
        private val accelConstraint: TrajectoryAccelerationConstraint =
            getAccelerationConstraint(MAX_ACCEL)

        fun getVelocityConstraint(
            maxVel: Double,
            maxAngularVel: Double,
            trackWidth: Double
        ): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(
                Arrays.asList(
                    AngularVelocityConstraint(maxAngularVel),
                    TankVelocityConstraint(maxVel, trackWidth)
                )
            )
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }
}