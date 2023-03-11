package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.geometry.Pose2d

class WaitSegment(pose: Pose2d, seconds: Double, markers: List<TrajectoryMarker>) :
    SequenceSegment(seconds, pose, pose, markers)