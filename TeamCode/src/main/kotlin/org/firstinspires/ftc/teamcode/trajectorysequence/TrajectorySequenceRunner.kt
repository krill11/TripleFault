package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.dashboard.FtcDashboard

@Config
class TrajectorySequenceRunner(
    follower: TrajectoryFollower,
    headingPIDCoefficients: PIDCoefficients?,
    voltageSensor: VoltageSensor,
    lastDriveEncPositions: List<Integer>,
    lastDriveEncVels: List<Integer>,
    lastTrackingEncPositions: List<Integer>,
    lastTrackingEncVels: List<Integer>
) {
    private val follower: TrajectoryFollower
    private val turnController: PIDFController
    private val clock: NanoClock
    private var currentTrajectorySequence: TrajectorySequence? = null
    private var currentSegmentStartTime = 0.0
    private var currentSegmentIndex = 0
    private var lastSegmentIndex = 0
    private var lastPoseError: Pose2d = Pose2d()
    var remainingMarkers: List<TrajectoryMarker> = ArrayList()
    private val dashboard: FtcDashboard
    private val poseHistory: LinkedList<Pose2d> = LinkedList()
    private val voltageSensor: VoltageSensor
    private val lastDriveEncPositions: List<Integer>
    private val lastDriveEncVels: List<Integer>
    private val lastTrackingEncPositions: List<Integer>
    private val lastTrackingEncVels: List<Integer>

    init {
        this.follower = follower
        turnController = PIDFController(headingPIDCoefficients)
        turnController.setInputBounds(0, 2 * Math.PI)
        this.voltageSensor = voltageSensor
        this.lastDriveEncPositions = lastDriveEncPositions
        this.lastDriveEncVels = lastDriveEncVels
        this.lastTrackingEncPositions = lastTrackingEncPositions
        this.lastTrackingEncVels = lastTrackingEncVels
        clock = NanoClock.system()
        dashboard = FtcDashboard.getInstance()
        dashboard.setTelemetryTransmissionInterval(25)
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence?) {
        currentTrajectorySequence = trajectorySequence
        currentSegmentStartTime = clock.seconds()
        currentSegmentIndex = 0
        lastSegmentIndex = -1
    }

    @Nullable
    fun update(poseEstimate: Pose2d, poseVelocity: Pose2d?): DriveSignal? {
        var targetPose: Pose2d? = null
        var driveSignal: DriveSignal? = null
        val packet = TelemetryPacket()
        val fieldOverlay: Canvas = packet.fieldOverlay()
        var currentSegment: SequenceSegment? = null
        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                for (marker in remainingMarkers) {
                    marker.getCallback().onMarkerReached()
                }
                remainingMarkers.clear()
                currentTrajectorySequence = null
            }
            if (currentTrajectorySequence == null) return DriveSignal()
            val now: Double = clock.seconds()
            val isNewTransition = currentSegmentIndex != lastSegmentIndex
            currentSegment = currentTrajectorySequence.get(currentSegmentIndex)
            if (isNewTransition) {
                currentSegmentStartTime = now
                lastSegmentIndex = currentSegmentIndex
                for (marker in remainingMarkers) {
                    marker.getCallback().onMarkerReached()
                }
                remainingMarkers.clear()
                remainingMarkers.addAll(currentSegment.getMarkers())
                Collections.sort(remainingMarkers) { t1, t2 ->
                    Double.compare(
                        t1.getTime(),
                        t2.getTime()
                    )
                }
            }
            val deltaTime = now - currentSegmentStartTime
            if (currentSegment is TrajectorySegment) {
                val currentTrajectory: Trajectory =
                    (currentSegment as TrajectorySegment?).getTrajectory()
                if (isNewTransition) follower.followTrajectory(currentTrajectory)
                if (!follower.isFollowing()) {
                    currentSegmentIndex++
                    driveSignal = DriveSignal()
                } else {
                    driveSignal = follower.update(poseEstimate, poseVelocity)
                    lastPoseError = follower.getLastError()
                }
                targetPose = currentTrajectory.get(deltaTime)
            } else if (currentSegment is TurnSegment) {
                val targetState: MotionState =
                    (currentSegment as TurnSegment?).getMotionProfile().get(deltaTime)
                turnController.setTargetPosition(targetState.getX())
                val correction: Double = turnController.update(poseEstimate.getHeading())
                val targetOmega: Double = targetState.getV()
                val targetAlpha: Double = targetState.getA()
                lastPoseError = Pose2d(0, 0, turnController.getLastError())
                val startPose: Pose2d = currentSegment.getStartPose()
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX())
                driveSignal = DriveSignal(
                    Pose2d(0, 0, targetOmega + correction),
                    Pose2d(0, 0, targetAlpha)
                )
                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++
                    driveSignal = DriveSignal()
                }
            } else if (currentSegment is WaitSegment) {
                lastPoseError = Pose2d()
                targetPose = currentSegment.getStartPose()
                driveSignal = DriveSignal()
                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++
                }
            }
            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers[0].getTime()) {
                remainingMarkers[0].getCallback().onMarkerReached()
                remainingMarkers.remove(0)
            }
        }
        poseHistory.add(poseEstimate)
        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }
        val NOMINAL_VOLTAGE = 12.0
        val voltage: Double = voltageSensor.getVoltage()
        if (driveSignal != null && !DriveConstants.RUN_USING_ENCODER) {
            driveSignal = DriveSignal(
                driveSignal.getVel().times(NOMINAL_VOLTAGE / voltage),
                driveSignal.getAccel().times(NOMINAL_VOLTAGE / voltage)
            )
        }
        if (targetPose != null) {
            LogFiles.record(
                targetPose,
                poseEstimate,
                voltage,
                lastDriveEncPositions,
                lastDriveEncVels,
                lastTrackingEncPositions,
                lastTrackingEncVels
            )
        }
        packet.put("x", poseEstimate.getX())
        packet.put("y", poseEstimate.getY())
        packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()))
        packet.put("xError", getLastPoseError().getX())
        packet.put("yError", getLastPoseError().getY())
        packet.put("headingError (deg)", Math.toDegrees(getLastPoseError().getHeading()))
        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate)
        dashboard.sendTelemetryPacket(packet)
        return driveSignal
    }

    private fun draw(
        fieldOverlay: Canvas,
        sequence: TrajectorySequence?, currentSegment: SequenceSegment?,
        targetPose: Pose2d?, poseEstimate: Pose2d
    ) {
        if (sequence != null) {
            for (i in 0 until sequence.size()) {
                val segment: SequenceSegment = sequence.get(i)
                if (segment is TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY)
                    DashboardUtil.drawSampledPath(
                        fieldOverlay,
                        (segment as TrajectorySegment).getTrajectory().getPath()
                    )
                } else if (segment is TurnSegment) {
                    val pose: Pose2d = segment.getStartPose()
                    fieldOverlay.setFill(COLOR_INACTIVE_TURN)
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2)
                } else if (segment is WaitSegment) {
                    val pose: Pose2d = segment.getStartPose()
                    fieldOverlay.setStrokeWidth(1)
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT)
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3)
                }
            }
        }
        if (currentSegment != null) {
            if (currentSegment is TrajectorySegment) {
                val currentTrajectory: Trajectory =
                    (currentSegment as TrajectorySegment).getTrajectory()
                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY)
                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath())
            } else if (currentSegment is TurnSegment) {
                val pose: Pose2d = currentSegment.getStartPose()
                fieldOverlay.setFill(COLOR_ACTIVE_TURN)
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3)
            } else if (currentSegment is WaitSegment) {
                val pose: Pose2d = currentSegment.getStartPose()
                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT)
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3)
            }
        }
        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.setStroke("#4CAF50")
            DashboardUtil.drawRobot(fieldOverlay, targetPose)
        }
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate)
    }

    fun getLastPoseError(): Pose2d {
        return lastPoseError
    }

    val isBusy: Boolean
        get() = currentTrajectorySequence != null

    companion object {
        var COLOR_INACTIVE_TRAJECTORY = "#4caf507a"
        var COLOR_INACTIVE_TURN = "#7c4dff7a"
        var COLOR_INACTIVE_WAIT = "#dd2c007a"
        var COLOR_ACTIVE_TRAJECTORY = "#4CAF50"
        var COLOR_ACTIVE_TURN = "#7c4dff"
        var COLOR_ACTIVE_WAIT = "#dd2c00"
        var POSE_HISTORY_LIMIT = 100
    }
}