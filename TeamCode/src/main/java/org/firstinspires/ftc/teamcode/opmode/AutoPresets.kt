package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.subsystem.HuskySubsystem.SpikeLocation

enum class AutoPresets(val index: Int) {
    LD_BLUE(0),
    SD_BLUE(1),
    LD_RED(2),
    SD_RED(3);

    public companion object {
        @JvmField
        public var LD_RED_STARTPOS: Pose2d = Pose2d(-37.0, -62.0, Math.toRadians(180.0))

        @JvmField
        public var SD_RED_STARTPOS: Pose2d = Pose2d(15.0, -62.0, Math.toRadians(270.0))

        @JvmField
        public var LD_BLUE_STARTPOS: Pose2d = Pose2d(-37.0, 62.0, Math.toRadians(180.0))

        @JvmField
        public var SD_BLUE_STARTPOS: Pose2d = Pose2d(15.0, 62.0, Math.toRadians(270.0))

        @JvmStatic
        public fun getSpikeLocation(alliance: Alliance, distance: Distance, spikeLoc: SpikeLocation): Pose2d? {
            // hacky af
            // TODO: We're missing spike movement locations for all RED and all LONG BLUE
            val switchString = alliance.toString() + distance.toString() + spikeLoc.toString()
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.LEFT)) return Pose2d(
                22.0,
                39.0,
                Math.toRadians(270.0)
            )
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.CENTER)) return Pose2d(
                21.0,
                24.0,
                Math.toRadians(180.0)
            )
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.RIGHT)) return Pose2d(
                0.0,
                37.0,
                Math.toRadians(270.0)
            )
            return null
        }

        private fun name(alliance: Alliance, distance: Distance, spikeLoc: SpikeLocation): String {
            return alliance.toString() + distance.toString() + spikeLoc.toString()
        }
    }

    val distance: Distance
        get() = if ((this.toString().contains("LD"))) Distance.LONG else Distance.SHORT
    val alliance: Alliance
        get() = if ((this.toString().contains("BLUE"))) Alliance.BLUE else Alliance.RED

    enum class Distance {
        SHORT,
        LONG
    }

    enum class Alliance {
        BLUE,
        RED
    }

    enum class Type {
        SPIKE,
        FOLLOW,
        BACKBOARD
    }


}
