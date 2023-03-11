package org.firstinspires.ftc.teamcode.util

import android.annotation.SuppressLint
import android.content.Context
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.fasterxml.jackson.databind.ObjectWriter
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.WebHandlerManager
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer
import java.io.File
import java.io.FileInputStream
import java.io.IOException
import java.text.DateFormat
import java.text.SimpleDateFormat
import java.util.ArrayList
import java.util.Arrays
import java.util.Date
import java.util.List
import java.util.Objects
import fi.iki.elonen.NanoHTTPD

object LogFiles {
    private val ROOT: File = File(AppUtil.ROOT_FOLDER + "/RoadRunner/logs/")
    var log = LogFile("uninitialized")
    fun record(
        targetPose: Pose2d,
        pose: Pose2d,
        voltage: Double,
        lastDriveEncPositions: List<Integer?>,
        lastDriveEncVels: List<Integer?>,
        lastTrackingEncPositions: List<Integer?>,
        lastTrackingEncVels: List<Integer?>
    ) {
        val nsTime: Long = System.nanoTime()
        if (nsTime - log.nsStart > 3 * 60 * 1000000000L) {
            return
        }
        log.nsTimes.add(nsTime)
        log.targetXs.add(targetPose.getX())
        log.targetYs.add(targetPose.getY())
        log.targetHeadings.add(targetPose.getHeading())
        log.xs.add(pose.getX())
        log.ys.add(pose.getY())
        log.headings.add(pose.getHeading())
        log.voltages.add(voltage)
        while (log.driveEncPositions.size() < lastDriveEncPositions.size()) {
            log.driveEncPositions.add(ArrayList())
        }
        while (log.driveEncVels.size() < lastDriveEncVels.size()) {
            log.driveEncVels.add(ArrayList())
        }
        while (log.trackingEncPositions.size() < lastTrackingEncPositions.size()) {
            log.trackingEncPositions.add(ArrayList())
        }
        while (log.trackingEncVels.size() < lastTrackingEncVels.size()) {
            log.trackingEncVels.add(ArrayList())
        }
        for (i in 0 until lastDriveEncPositions.size()) {
            log.driveEncPositions[i].add(
                lastDriveEncPositions[i]
            )
        }
        for (i in 0 until lastDriveEncVels.size()) {
            log.driveEncVels[i].add(lastDriveEncVels[i])
        }
        for (i in 0 until lastTrackingEncPositions.size()) {
            log.trackingEncPositions[i].add(
                lastTrackingEncPositions[i]
            )
        }
        for (i in 0 until lastTrackingEncVels.size()) {
            log.trackingEncVels[i].add(
                lastTrackingEncVels[i]
            )
        }
    }

    private val notifHandler: OpModeManagerNotifier.Notifications = object : Notifications() {
        @SuppressLint("SimpleDateFormat")
        val dateFormat: DateFormat = SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS")
        val jsonWriter: ObjectWriter = ObjectMapper(JsonFactory())
            .writerWithDefaultPrettyPrinter()

        @Override
        fun onOpModePreInit(opMode: OpMode) {
            log = LogFile(opMode.getClass().getCanonicalName())

            // clean up old files
            val fs: Array<File> = Objects.requireNonNull(ROOT.listFiles())
            Arrays.sort(fs) { a, b -> Long.compare(a.lastModified(), b.lastModified()) }
            var totalSizeBytes: Long = 0
            for (f in fs) {
                totalSizeBytes += f.length()
            }
            var i = 0
            while (i < fs.size && totalSizeBytes >= 32 * 1000 * 1000) {
                totalSizeBytes -= fs[i].length()
                if (!fs[i].delete()) {
                    RobotLog.setGlobalErrorMsg("Unable to delete file " + fs[i].getAbsolutePath())
                }
                ++i
            }
        }

        @Override
        fun onOpModePreStart(opMode: OpMode?) {
            log.nsStart = System.nanoTime()
        }

        @Override
        fun onOpModePostStop(opMode: OpMode) {
            log.nsStop = System.nanoTime()
            if (opMode !is OpModeManagerImpl.DefaultOpMode) {
                ROOT.mkdirs()
                val filename: String =
                    dateFormat.format(Date(log.msInit)) + "__" + opMode.getClass()
                        .getSimpleName() + ".json"
                val file = File(ROOT, filename)
                try {
                    jsonWriter.writeValue(file, log)
                } catch (e: IOException) {
                    RobotLog.setGlobalErrorMsg(
                        RuntimeException(e),
                        "Unable to write data to " + file.getAbsolutePath()
                    )
                }
            }
        }
    }

    @WebHandlerRegistrar
    fun registerRoutes(context: Context?, manager: WebHandlerManager) {
        ROOT.mkdirs()

        // op mode manager only stores a weak reference, so we need to keep notifHandler alive ourselves
        // don't use @OnCreateEventLoop because it's unreliable
        OpModeManagerImpl.getOpModeManagerOfActivity(
            AppUtil.getInstance().getActivity()
        ).registerListener(notifHandler)
        manager.register("/logs") { session ->
            val sb = StringBuilder()
            sb.append("<!doctype html><html><head><title>Logs</title></head><body><ul>")
            val fs: Array<File> = Objects.requireNonNull(ROOT.listFiles())
            Arrays.sort(fs) { a, b -> Long.compare(b.lastModified(), a.lastModified()) }
            for (f in fs) {
                sb.append("<li><a href=\"/logs/download?file=")
                sb.append(f.getName())
                sb.append("\" download=\"")
                sb.append(f.getName())
                sb.append("\">")
                sb.append(f.getName())
                sb.append("</a></li>")
            }
            sb.append("</ul></body></html>")
            NanoHTTPD.newFixedLengthResponse(
                NanoHTTPD.Response.Status.OK,
                NanoHTTPD.MIME_HTML, sb.toString()
            )
        }
        manager.register("/logs/download") { session ->
            val pairs: Array<String> = session.getQueryParameterString().split("&")
            if (pairs.size != 1) {
                return@register NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.BAD_REQUEST,
                    NanoHTTPD.MIME_PLAINTEXT, "expected one query parameter, got " + pairs.size
                )
            }
            val parts: Array<String> = pairs[0].split("=")
            if (!parts[0].equals("file")) {
                return@register NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.BAD_REQUEST,
                    NanoHTTPD.MIME_PLAINTEXT, "expected file query parameter, got " + parts[0]
                )
            }
            val f = File(ROOT, parts[1])
            if (!f.exists()) {
                return@register NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.NOT_FOUND,
                    NanoHTTPD.MIME_PLAINTEXT, "file $f doesn't exist"
                )
            }
            NanoHTTPD.newChunkedResponse(
                NanoHTTPD.Response.Status.OK,
                "application/json", FileInputStream(f)
            )
        }
    }

    class LogFile(var opModeName: String) {
        var version = "quickstart1 v2"
        var msInit: Long = System.currentTimeMillis()
        var nsInit: Long = System.nanoTime()
        var nsStart: Long = 0
        var nsStop: Long = 0
        var ticksPerRev: Double = DriveConstants.TICKS_PER_REV
        var maxRpm: Double = DriveConstants.MAX_RPM
        var runUsingEncoder: Boolean = DriveConstants.RUN_USING_ENCODER
        var motorP: Double = DriveConstants.MOTOR_VELO_PID.p
        var motorI: Double = DriveConstants.MOTOR_VELO_PID.i
        var motorD: Double = DriveConstants.MOTOR_VELO_PID.d
        var motorF: Double = DriveConstants.MOTOR_VELO_PID.f
        var wheelRadius: Double = DriveConstants.WHEEL_RADIUS
        var gearRatio: Double = DriveConstants.GEAR_RATIO
        var trackWidth: Double = DriveConstants.TRACK_WIDTH
        var kV: Double = DriveConstants.kV
        var kA: Double = DriveConstants.kA
        var kStatic: Double = DriveConstants.kStatic
        var maxVel: Double = DriveConstants.MAX_VEL
        var maxAccel: Double = DriveConstants.MAX_ACCEL
        var maxAngVel: Double = DriveConstants.MAX_ANG_VEL
        var maxAngAccel: Double = DriveConstants.MAX_ANG_ACCEL
        var mecTransP: Double = SampleMecanumDrive.TRANSLATIONAL_PID.kP
        var mecTransI: Double = SampleMecanumDrive.TRANSLATIONAL_PID.kI
        var mecTransD: Double = SampleMecanumDrive.TRANSLATIONAL_PID.kD
        var mecHeadingP: Double = SampleMecanumDrive.HEADING_PID.kP
        var mecHeadingI: Double = SampleMecanumDrive.HEADING_PID.kI
        var mecHeadingD: Double = SampleMecanumDrive.HEADING_PID.kD
        var mecLateralMultiplier: Double = SampleMecanumDrive.LATERAL_MULTIPLIER
        var tankAxialP: Double = SampleTankDrive.AXIAL_PID.kP
        var tankAxialI: Double = SampleTankDrive.AXIAL_PID.kI
        var tankAxialD: Double = SampleTankDrive.AXIAL_PID.kD
        var tankCrossTrackP: Double = SampleTankDrive.CROSS_TRACK_PID.kP
        var tankCrossTrackI: Double = SampleTankDrive.CROSS_TRACK_PID.kI
        var tankCrossTrackD: Double = SampleTankDrive.CROSS_TRACK_PID.kD
        var tankHeadingP: Double = SampleTankDrive.HEADING_PID.kP
        var tankHeadingI: Double = SampleTankDrive.HEADING_PID.kI
        var tankHeadingD: Double = SampleTankDrive.HEADING_PID.kD
        var trackingTicksPerRev: Double = StandardTrackingWheelLocalizer.TICKS_PER_REV
        var trackingWheelRadius: Double = StandardTrackingWheelLocalizer.WHEEL_RADIUS
        var trackingGearRatio: Double = StandardTrackingWheelLocalizer.GEAR_RATIO
        var trackingLateralDistance: Double = StandardTrackingWheelLocalizer.LATERAL_DISTANCE
        var trackingForwardOffset: Double = StandardTrackingWheelLocalizer.FORWARD_OFFSET
        var LOGO_FACING_DIR: RevHubOrientationOnRobot.LogoFacingDirection =
            DriveConstants.LOGO_FACING_DIR
        var USB_FACING_DIR: RevHubOrientationOnRobot.UsbFacingDirection =
            DriveConstants.USB_FACING_DIR
        var nsTimes: List<Long> = ArrayList()
        var targetXs: List<Double> = ArrayList()
        var targetYs: List<Double> = ArrayList()
        var targetHeadings: List<Double> = ArrayList()
        var xs: List<Double> = ArrayList()
        var ys: List<Double> = ArrayList()
        var headings: List<Double> = ArrayList()
        var voltages: List<Double> = ArrayList()
        var driveEncPositions: List<List<Integer>> = ArrayList()
        var driveEncVels: List<List<Integer>> = ArrayList()
        var trackingEncPositions: List<List<Integer>> = ArrayList()
        var trackingEncVels: List<List<Integer>> = ArrayList()
    }
}