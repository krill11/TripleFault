package org.firstinspires.ftc.teamcode.trajectorysequence

import com.acmerobotics.roadrunner.geometry.Pose2d

class TrajectorySequenceBuilder(
    startPose: Pose2d,
    startTangent: Double?,
    baseVelConstraint: TrajectoryVelocityConstraint,
    baseAccelConstraint: TrajectoryAccelerationConstraint,
    baseTurnConstraintMaxAngVel: Double,
    baseTurnConstraintMaxAngAccel: Double
) {
    private val resolution = 0.25
    private val baseVelConstraint: TrajectoryVelocityConstraint
    private val baseAccelConstraint: TrajectoryAccelerationConstraint
    private var currentVelConstraint: TrajectoryVelocityConstraint
    private var currentAccelConstraint: TrajectoryAccelerationConstraint
    private val baseTurnConstraintMaxAngVel: Double
    private val baseTurnConstraintMaxAngAccel: Double
    private var currentTurnConstraintMaxAngVel: Double
    private var currentTurnConstraintMaxAngAccel: Double
    private val sequenceSegments: List<SequenceSegment>
    private val temporalMarkers: List<TemporalMarker>
    private val displacementMarkers: List<DisplacementMarker>
    private val spatialMarkers: List<SpatialMarker>
    private var lastPose: Pose2d
    private var tangentOffset: Double
    private var setAbsoluteTangent: Boolean
    private var absoluteTangent: Double
    private var currentTrajectoryBuilder: TrajectoryBuilder?
    private var currentDuration: Double
    private var currentDisplacement: Double
    private var lastDurationTraj: Double
    private var lastDisplacementTraj: Double

    init {
        this.baseVelConstraint = baseVelConstraint
        this.baseAccelConstraint = baseAccelConstraint
        currentVelConstraint = baseVelConstraint
        currentAccelConstraint = baseAccelConstraint
        this.baseTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        this.baseTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        sequenceSegments = ArrayList()
        temporalMarkers = ArrayList()
        displacementMarkers = ArrayList()
        spatialMarkers = ArrayList()
        lastPose = startPose
        tangentOffset = 0.0
        setAbsoluteTangent = startTangent != null
        absoluteTangent = startTangent ?: 0.0
        currentTrajectoryBuilder = null
        currentDuration = 0.0
        currentDisplacement = 0.0
        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
    }

    constructor(
        startPose: Pose2d,
        baseVelConstraint: TrajectoryVelocityConstraint,
        baseAccelConstraint: TrajectoryAccelerationConstraint,
        baseTurnConstraintMaxAngVel: Double,
        baseTurnConstraintMaxAngAccel: Double
    ) : this(
        startPose, null,
        baseVelConstraint, baseAccelConstraint,
        baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
    ) {
    }

    fun lineTo(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineTo(
                endPosition,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun lineTo(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineTo(
                endPosition,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun lineToConstantHeading(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToConstantHeading(
                endPosition,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun lineToConstantHeading(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToConstantHeading(
                endPosition,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun lineToLinearHeading(endPose: Pose2d?): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToLinearHeading(
                endPose,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun lineToLinearHeading(
        endPose: Pose2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToLinearHeading(
                endPose,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun lineToSplineHeading(endPose: Pose2d?): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToSplineHeading(
                endPose,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun lineToSplineHeading(
        endPose: Pose2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.lineToSplineHeading(
                endPose,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun strafeTo(endPosition: Vector2d?): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeTo(
                endPosition,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun strafeTo(
        endPosition: Vector2d?,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeTo(
                endPosition,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun forward(distance: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.forward(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun forward(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.forward(
                distance,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun back(distance: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.back(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun back(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.back(
                distance,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun strafeLeft(distance: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeLeft(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun strafeLeft(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeLeft(
                distance,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun strafeRight(distance: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeRight(
                distance,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun strafeRight(
        distance: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.strafeRight(
                distance,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun splineTo(endPosition: Vector2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineTo(
                endPosition,
                endHeading,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun splineTo(
        endPosition: Vector2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineTo(
                endPosition,
                endHeading,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun splineToConstantHeading(
        endPosition: Vector2d?,
        endHeading: Double
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToConstantHeading(
                endPosition,
                endHeading,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun splineToConstantHeading(
        endPosition: Vector2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToConstantHeading(
                endPosition,
                endHeading,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun splineToLinearHeading(endPose: Pose2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToLinearHeading(
                endPose,
                endHeading,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun splineToLinearHeading(
        endPose: Pose2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToLinearHeading(
                endPose,
                endHeading,
                velConstraint,
                accelConstraint
            )
        })
    }

    fun splineToSplineHeading(endPose: Pose2d?, endHeading: Double): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToSplineHeading(
                endPose,
                endHeading,
                currentVelConstraint,
                currentAccelConstraint
            )
        })
    }

    fun splineToSplineHeading(
        endPose: Pose2d?,
        endHeading: Double,
        velConstraint: TrajectoryVelocityConstraint?,
        accelConstraint: TrajectoryAccelerationConstraint?
    ): TrajectorySequenceBuilder {
        return addPath(AddPathCallback {
            currentTrajectoryBuilder.splineToSplineHeading(
                endPose,
                endHeading,
                velConstraint,
                accelConstraint
            )
        })
    }

    private fun addPath(callback: AddPathCallback): TrajectorySequenceBuilder {
        if (currentTrajectoryBuilder == null) newPath()
        try {
            callback.run()
        } catch (e: PathContinuityViolationException) {
            newPath()
            callback.run()
        }
        val builtTraj: Trajectory = currentTrajectoryBuilder.build()
        val durationDifference: Double = builtTraj.duration() - lastDurationTraj
        val displacementDifference: Double = builtTraj.getPath().length() - lastDisplacementTraj
        lastPose = builtTraj.end()
        currentDuration += durationDifference
        currentDisplacement += displacementDifference
        lastDurationTraj = builtTraj.duration()
        lastDisplacementTraj = builtTraj.getPath().length()
        return this
    }

    fun setTangent(tangent: Double): TrajectorySequenceBuilder {
        setAbsoluteTangent = true
        absoluteTangent = tangent
        pushPath()
        return this
    }

    private fun setTangentOffset(offset: Double): TrajectorySequenceBuilder {
        setAbsoluteTangent = false
        tangentOffset = offset
        pushPath()
        return this
    }

    fun setReversed(reversed: Boolean): TrajectorySequenceBuilder {
        return if (reversed) setTangentOffset(Math.toRadians(180.0)) else setTangentOffset(0.0)
    }

    fun setConstraints(
        velConstraint: TrajectoryVelocityConstraint,
        accelConstraint: TrajectoryAccelerationConstraint
    ): TrajectorySequenceBuilder {
        currentVelConstraint = velConstraint
        currentAccelConstraint = accelConstraint
        return this
    }

    fun resetConstraints(): TrajectorySequenceBuilder {
        currentVelConstraint = baseVelConstraint
        currentAccelConstraint = baseAccelConstraint
        return this
    }

    fun setVelConstraint(velConstraint: TrajectoryVelocityConstraint): TrajectorySequenceBuilder {
        currentVelConstraint = velConstraint
        return this
    }

    fun resetVelConstraint(): TrajectorySequenceBuilder {
        currentVelConstraint = baseVelConstraint
        return this
    }

    fun setAccelConstraint(accelConstraint: TrajectoryAccelerationConstraint): TrajectorySequenceBuilder {
        currentAccelConstraint = accelConstraint
        return this
    }

    fun resetAccelConstraint(): TrajectorySequenceBuilder {
        currentAccelConstraint = baseAccelConstraint
        return this
    }

    fun setTurnConstraint(maxAngVel: Double, maxAngAccel: Double): TrajectorySequenceBuilder {
        currentTurnConstraintMaxAngVel = maxAngVel
        currentTurnConstraintMaxAngAccel = maxAngAccel
        return this
    }

    fun resetTurnConstraint(): TrajectorySequenceBuilder {
        currentTurnConstraintMaxAngVel = baseTurnConstraintMaxAngVel
        currentTurnConstraintMaxAngAccel = baseTurnConstraintMaxAngAccel
        return this
    }

    fun addTemporalMarker(callback: MarkerCallback?): TrajectorySequenceBuilder {
        return this.addTemporalMarker(currentDuration, callback)
    }

    fun UNSTABLE_addTemporalMarkerOffset(
        offset: Double,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        return this.addTemporalMarker(currentDuration + offset, callback)
    }

    fun addTemporalMarker(time: Double, callback: MarkerCallback?): TrajectorySequenceBuilder {
        return this.addTemporalMarker(0.0, time, callback)
    }

    fun addTemporalMarker(
        scale: Double,
        offset: Double,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        return this.addTemporalMarker({ time -> scale * time + offset }, callback)
    }

    fun addTemporalMarker(
        time: TimeProducer?,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        temporalMarkers.add(TemporalMarker(time, callback))
        return this
    }

    fun addSpatialMarker(point: Vector2d?, callback: MarkerCallback?): TrajectorySequenceBuilder {
        spatialMarkers.add(SpatialMarker(point, callback))
        return this
    }

    fun addDisplacementMarker(callback: MarkerCallback?): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(currentDisplacement, callback)
    }

    fun UNSTABLE_addDisplacementMarkerOffset(
        offset: Double,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(currentDisplacement + offset, callback)
    }

    fun addDisplacementMarker(
        displacement: Double,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        return this.addDisplacementMarker(0.0, displacement, callback)
    }

    fun addDisplacementMarker(
        scale: Double,
        offset: Double,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        return addDisplacementMarker({ displacement -> scale * displacement + offset }, callback)
    }

    fun addDisplacementMarker(
        displacement: DisplacementProducer?,
        callback: MarkerCallback?
    ): TrajectorySequenceBuilder {
        displacementMarkers.add(DisplacementMarker(displacement, callback))
        return this
    }

    @JvmOverloads
    fun turn(
        angle: Double,
        maxAngVel: Double = currentTurnConstraintMaxAngVel,
        maxAngAccel: Double = currentTurnConstraintMaxAngAccel
    ): TrajectorySequenceBuilder {
        pushPath()
        val turnProfile: MotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(lastPose.getHeading(), 0.0, 0.0, 0.0),
            MotionState(lastPose.getHeading() + angle, 0.0, 0.0, 0.0),
            maxAngVel,
            maxAngAccel
        )
        sequenceSegments.add(TurnSegment(lastPose, angle, turnProfile, Collections.emptyList()))
        lastPose = Pose2d(
            lastPose.getX(), lastPose.getY(),
            Angle.norm(lastPose.getHeading() + angle)
        )
        currentDuration += turnProfile.duration()
        return this
    }

    fun waitSeconds(seconds: Double): TrajectorySequenceBuilder {
        pushPath()
        sequenceSegments.add(WaitSegment(lastPose, seconds, Collections.emptyList()))
        currentDuration += seconds
        return this
    }

    fun addTrajectory(trajectory: Trajectory?): TrajectorySequenceBuilder {
        pushPath()
        sequenceSegments.add(TrajectorySegment(trajectory))
        return this
    }

    private fun pushPath() {
        if (currentTrajectoryBuilder != null) {
            val builtTraj: Trajectory = currentTrajectoryBuilder.build()
            sequenceSegments.add(TrajectorySegment(builtTraj))
        }
        currentTrajectoryBuilder = null
    }

    private fun newPath() {
        if (currentTrajectoryBuilder != null) pushPath()
        lastDurationTraj = 0.0
        lastDisplacementTraj = 0.0
        val tangent =
            if (setAbsoluteTangent) absoluteTangent else Angle.norm(lastPose.getHeading() + tangentOffset)
        currentTrajectoryBuilder = TrajectoryBuilder(
            lastPose,
            tangent,
            currentVelConstraint,
            currentAccelConstraint,
            resolution
        )
    }

    fun build(): TrajectorySequence {
        pushPath()
        val globalMarkers: List<TrajectoryMarker> = convertMarkersToGlobal(
            sequenceSegments,
            temporalMarkers, displacementMarkers, spatialMarkers
        )
        return TrajectorySequence(
            projectGlobalMarkersToLocalSegments(
                globalMarkers,
                sequenceSegments
            )
        )
    }

    private fun convertMarkersToGlobal(
        sequenceSegments: List<SequenceSegment>,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): List<TrajectoryMarker> {
        val trajectoryMarkers: ArrayList<TrajectoryMarker> = ArrayList()

        // Convert temporal markers
        for (marker in temporalMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(
                    marker.getProducer().produce(currentDuration),
                    marker.getCallback()
                )
            )
        }

        // Convert displacement markers
        for (marker in displacementMarkers) {
            val time = displacementToTime(
                sequenceSegments,
                marker.getProducer().produce(currentDisplacement)
            )
            trajectoryMarkers.add(
                TrajectoryMarker(
                    time,
                    marker.getCallback()
                )
            )
        }

        // Convert spatial markers
        for (marker in spatialMarkers) {
            trajectoryMarkers.add(
                TrajectoryMarker(
                    pointToTime(sequenceSegments, marker.getPoint()),
                    marker.getCallback()
                )
            )
        }
        return trajectoryMarkers
    }

    private fun projectGlobalMarkersToLocalSegments(
        markers: List<TrajectoryMarker>,
        sequenceSegments: List<SequenceSegment>
    ): List<SequenceSegment> {
        if (sequenceSegments.isEmpty()) return Collections.emptyList()
        var totalSequenceDuration = 0.0
        for (segment in sequenceSegments) {
            totalSequenceDuration += segment.getDuration()
        }
        for (marker in markers) {
            var segment: SequenceSegment? = null
            var segmentIndex = 0
            var segmentOffsetTime = 0.0
            var currentTime = 0.0
            for (i in 0 until sequenceSegments.size()) {
                val seg: SequenceSegment = sequenceSegments[i]
                val markerTime: Double = Math.min(marker.getTime(), totalSequenceDuration)
                if (currentTime + seg.getDuration() >= markerTime) {
                    segment = seg
                    segmentIndex = i
                    segmentOffsetTime = markerTime - currentTime
                    break
                } else {
                    currentTime += seg.getDuration()
                }
            }
            var newSegment: SequenceSegment? = null
            if (segment is WaitSegment) {
                val newMarkers: List<TrajectoryMarker> = ArrayList(segment.getMarkers())
                newMarkers.addAll(sequenceSegments[segmentIndex].getMarkers())
                newMarkers.add(TrajectoryMarker(segmentOffsetTime, marker.getCallback()))
                val thisSegment: WaitSegment? = segment as WaitSegment?
                newSegment =
                    WaitSegment(thisSegment.getStartPose(), thisSegment.getDuration(), newMarkers)
            } else if (segment is TurnSegment) {
                val newMarkers: List<TrajectoryMarker> = ArrayList(segment.getMarkers())
                newMarkers.addAll(sequenceSegments[segmentIndex].getMarkers())
                newMarkers.add(TrajectoryMarker(segmentOffsetTime, marker.getCallback()))
                val thisSegment: TurnSegment? = segment as TurnSegment?
                newSegment = TurnSegment(
                    thisSegment.getStartPose(),
                    thisSegment.getTotalRotation(),
                    thisSegment.getMotionProfile(),
                    newMarkers
                )
            } else if (segment is TrajectorySegment) {
                val thisSegment: TrajectorySegment? = segment as TrajectorySegment?
                val newMarkers: List<TrajectoryMarker> =
                    ArrayList(thisSegment.getTrajectory().getMarkers())
                newMarkers.add(TrajectoryMarker(segmentOffsetTime, marker.getCallback()))
                newSegment = TrajectorySegment(
                    Trajectory(
                        thisSegment.getTrajectory().getPath(),
                        thisSegment.getTrajectory().getProfile(),
                        newMarkers
                    )
                )
            }
            sequenceSegments.set(segmentIndex, newSegment)
        }
        return sequenceSegments
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
    private fun motionProfileDisplacementToTime(profile: MotionProfile, s: Double): Double {
        var tLo = 0.0
        var tHi: Double = profile.duration()
        while (Math.abs(tLo - tHi) >= 1e-6) {
            val tMid = 0.5 * (tLo + tHi)
            if (profile.get(tMid).getX() > s) {
                tHi = tMid
            } else {
                tLo = tMid
            }
        }
        return 0.5 * (tLo + tHi)
    }

    private fun displacementToTime(sequenceSegments: List<SequenceSegment>, s: Double): Double {
        var currentTime = 0.0
        var currentDisplacement = 0.0
        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment: TrajectorySegment = segment as TrajectorySegment
                val segmentLength: Double = thisSegment.getTrajectory().getPath().length()
                if (currentDisplacement + segmentLength > s) {
                    val target = s - currentDisplacement
                    val timeInSegment = motionProfileDisplacementToTime(
                        thisSegment.getTrajectory().getProfile(),
                        target
                    )
                    return currentTime + timeInSegment
                } else {
                    currentDisplacement += segmentLength
                    currentTime += thisSegment.getTrajectory().duration()
                }
            } else {
                currentTime += segment.getDuration()
            }
        }
        return 0.0
    }

    private fun pointToTime(sequenceSegments: List<SequenceSegment>, point: Vector2d): Double {
        class ComparingPoints(
            val distanceToPoint: Double,
            val totalDisplacement: Double,
            val thisPathDisplacement: Double
        )

        val projectedPoints: List<ComparingPoints> = ArrayList()
        for (segment in sequenceSegments) {
            if (segment is TrajectorySegment) {
                val thisSegment: TrajectorySegment = segment as TrajectorySegment
                val displacement: Double =
                    thisSegment.getTrajectory().getPath().project(point, 0.25)
                val projectedPoint: Vector2d =
                    thisSegment.getTrajectory().getPath().get(displacement).vec()
                val distanceToPoint: Double = point.minus(projectedPoint).norm()
                var totalDisplacement = 0.0
                for (comparingPoint in projectedPoints) {
                    totalDisplacement += comparingPoint.totalDisplacement
                }
                totalDisplacement += displacement
                projectedPoints.add(
                    ComparingPoints(
                        distanceToPoint,
                        displacement,
                        totalDisplacement
                    )
                )
            }
        }
        var closestPoint: ComparingPoints? = null
        for (comparingPoint in projectedPoints) {
            if (closestPoint == null) {
                closestPoint = comparingPoint
                continue
            }
            if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint) closestPoint =
                comparingPoint
        }
        return displacementToTime(sequenceSegments, closestPoint!!.thisPathDisplacement)
    }

    private interface AddPathCallback {
        fun run()
    }
}