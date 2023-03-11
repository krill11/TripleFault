package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.Misc
import java.util.HashMap
import java.util.Map

/**
 * Collection of utilites for interacting with Lynx modules.
 */
object LynxModuleUtil {
    private val MIN_VERSION = LynxFirmwareVersion(1, 8, 2)

    /**
     * Retrieve and parse Lynx module firmware version.
     * @param module Lynx module
     * @return parsed firmware version
     */
    fun getFirmwareVersion(module: LynxModule): LynxFirmwareVersion? {
        val versionString: String = module.getNullableFirmwareVersionString() ?: return null
        val parts: Array<String> = versionString.split("[ :,]+")
        return try {
            // note: for now, we ignore the hardware entry
            LynxFirmwareVersion(
                Integer.parseInt(parts[3]),
                Integer.parseInt(parts[5]),
                Integer.parseInt(parts[7])
            )
        } catch (e: NumberFormatException) {
            null
        }
    }

    /**
     * Ensure all of the Lynx modules attached to the robot satisfy the minimum requirement.
     * @param hardwareMap hardware map containing Lynx modules
     */
    fun ensureMinimumFirmwareVersion(hardwareMap: HardwareMap) {
        val outdatedModules: Map<String, LynxFirmwareVersion> = HashMap()
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            val version = getFirmwareVersion(module)
            if (version == null || version.compareTo(MIN_VERSION) < 0) {
                for (name in hardwareMap.getNamesOf(module)) {
                    outdatedModules.put(name, version)
                }
            }
        }
        if (outdatedModules.size() > 0) {
            val msgBuilder = StringBuilder()
            msgBuilder.append("One or more of the attached Lynx modules has outdated firmware\n")
            msgBuilder.append(
                Misc.formatInvariant(
                    "Mandatory minimum firmware version for Road Runner: %s\n",
                    MIN_VERSION.toString()
                )
            )
            for (entry in outdatedModules.entrySet()) {
                msgBuilder.append(
                    Misc.formatInvariant(
                        "\t%s: %s\n", entry.getKey(),
                        if (entry.getValue() == null) "Unknown" else entry.getValue().toString()
                    )
                )
            }
            throw LynxFirmwareVersionException(msgBuilder.toString())
        }
    }

    /**
     * Parsed representation of a Lynx module firmware version.
     */
    class LynxFirmwareVersion(val major: Int, val minor: Int, val eng: Int) :
        Comparable<LynxFirmwareVersion?> {
        @Override
        override fun equals(other: Object): Boolean {
            return if (other is LynxFirmwareVersion) {
                val otherVersion =
                    other as LynxFirmwareVersion
                major == otherVersion.major && minor == otherVersion.minor && eng == otherVersion.eng
            } else {
                false
            }
        }

        @Override
        operator fun compareTo(other: LynxFirmwareVersion): Int {
            val majorComp: Int = Integer.compare(major, other.major)
            return if (majorComp == 0) {
                val minorComp: Int = Integer.compare(minor, other.minor)
                if (minorComp == 0) {
                    Integer.compare(eng, other.eng)
                } else {
                    minorComp
                }
            } else {
                majorComp
            }
        }

        @Override
        override fun toString(): String {
            return Misc.formatInvariant("%d.%d.%d", major, minor, eng)
        }
    }

    /**
     * Exception indicating an outdated Lynx firmware version.
     */
    class LynxFirmwareVersionException(detailMessage: String?) : RuntimeException(detailMessage)
}