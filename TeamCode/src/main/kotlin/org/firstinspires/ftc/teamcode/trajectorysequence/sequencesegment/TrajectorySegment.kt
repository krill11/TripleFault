package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment

import com.acmerobotics.roadrunner.trajectory.Trajectory

class TrajectorySegment(trajectory: Trajectory) : SequenceSegment(
    trajectory.duration(),
    trajectory.start(),
    trajectory.end(),
    Collections.emptyList()
) {
    private val trajectory: Trajectory

    init {
        // Note: Markers are already stored in the `Trajectory` itself.
        // This class should not hold any markers
        this.trajectory = trajectory
    }

    fun getTrajectory(): Trajectory {
        return trajectory
    }
}