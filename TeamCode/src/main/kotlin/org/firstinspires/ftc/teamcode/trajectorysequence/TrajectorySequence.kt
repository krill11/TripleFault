package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.roadrunner.geometry.Pose2d

class TrajectorySequence(sequenceList: List<SequenceSegment?>) {
    private val sequenceList: List<SequenceSegment>

    init {
        if (sequenceList.size() === 0) throw EmptySequenceException()
        this.sequenceList = Collections.unmodifiableList(sequenceList)
    }

    fun start(): Pose2d {
        return sequenceList[0].getStartPose()
    }

    fun end(): Pose2d {
        return sequenceList[sequenceList.size() - 1].getEndPose()
    }

    fun duration(): Double {
        var total = 0.0
        for (segment in sequenceList) {
            total += segment.getDuration()
        }
        return total
    }

    operator fun get(i: Int): SequenceSegment {
        return sequenceList[i]
    }

    fun size(): Int {
        return sequenceList.size()
    }
}