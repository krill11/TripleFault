package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.File

/**
 * Utility functions for log files.
 */
object LoggingUtil {
    val ROAD_RUNNER_FOLDER: File = File(AppUtil.ROOT_FOLDER.absolutePath + "/RoadRunner/")
    private const val LOG_QUOTA = (25 * 1024 * 1024 // 25MB log quota for now
            ).toLong()

    private fun buildLogList(logFiles: ArrayList<File>, dir: File) {
        for (file in dir.listFiles()) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file)
            } else {
                logFiles.add(file)
            }
        }
    }

    private fun pruneLogsIfNecessary() {
        val logFiles: ArrayList<File> = ArrayList()
        buildLogList(logFiles, ROAD_RUNNER_FOLDER)
        logFiles.sortByDescending { file -> file.lastModified() }
        var dirSize: Long = 0
        for (file in logFiles) {
            dirSize += file.length()
        }
        while (dirSize > LOG_QUOTA) {
            if (logFiles.size === 0) break
            val fileToRemove: File = logFiles.removeAt(0)
            dirSize -= fileToRemove.length()
            fileToRemove.delete()
        }
    }

    /**
     * Obtain a log file with the provided name
     */
    fun getLogFile(name: String?): File {
        ROAD_RUNNER_FOLDER.mkdirs()
        pruneLogsIfNecessary()
        return File(ROAD_RUNNER_FOLDER, name)
    }
}