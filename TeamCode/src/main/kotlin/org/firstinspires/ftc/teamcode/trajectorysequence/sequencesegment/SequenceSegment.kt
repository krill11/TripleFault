package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d

abstract class SequenceSegment protected constructor(
    val duration: Double,
    startPose: Pose2d, endPose: Pose2d,
    markers: List<TrajectoryMarker>
) {
    private val startPose: Pose2d
    private val endPose: Pose2d
    private val markers: List<TrajectoryMarker>

    init {
        this.startPose = startPose
        this.endPose = endPose
        this.markers = markers
    }

    fun getStartPose(): Pose2d {
        return startPose
    }

    fun getEndPose(): Pose2d {
        return endPose
    }

    fun getMarkers(): List<TrajectoryMarker> {
        return markers
    }
}