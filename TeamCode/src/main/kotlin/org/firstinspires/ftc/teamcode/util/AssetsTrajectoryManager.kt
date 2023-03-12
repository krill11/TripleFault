package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.IOException
import java.io.InputStream

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
object AssetsTrajectoryManager {
    /**
     * Loads the group config.
     */
    fun loadGroupConfig(): TrajectoryGroupConfig? {
        return try {
            val inputStream: InputStream = AppUtil.getDefContext().getAssets().open(
                "trajectory/" + TrajectoryConfigManager.GROUP_FILENAME
            )
            TrajectoryConfigManager.loadGroupConfig(inputStream)
        } catch (e: IOException) {
            null
        }
    }

    /**
     * Loads a trajectory config with the given name.
     */
    fun loadConfig(name: String): TrajectoryConfig? {
        return try {
            val inputStream: InputStream = AppUtil.getDefContext().getAssets().open(
                "trajectory/$name.yaml"
            )
            TrajectoryConfigManager.loadConfig(inputStream)
        } catch (e: IOException) {
            null
        }
    }

    /**
     * Loads a trajectory builder with the given name.
     */
    fun loadBuilder(name: String): TrajectoryBuilder? {
        val groupConfig: TrajectoryGroupConfig? = loadGroupConfig()
        val config: TrajectoryConfig? = loadConfig(name)
        return if (groupConfig == null || config == null) {
            null
        } else config.toTrajectoryBuilder(
            groupConfig
        )
    }

    /**
     * Loads a trajectory with the given name.
     */
    fun load(name: String): Trajectory? {
        return loadBuilder(name)?.build()
    }
}