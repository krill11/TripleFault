package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d

class TurnSegment(
    startPose: Pose2d,
    val totalRotation: Double,
    motionProfile: MotionProfile,
    markers: List<TrajectoryMarker>
) : SequenceSegment(
    motionProfile.duration(),
    startPose,
    Pose2d(
        startPose.getX(), startPose.getY(),
        Angle.norm(startPose.getHeading() + totalRotation)
    ),
    markers
) {
    private val motionProfile: MotionProfile

    init {
        this.motionProfile = motionProfile
    }

    fun getMotionProfile(): MotionProfile {
        return motionProfile
    }
}