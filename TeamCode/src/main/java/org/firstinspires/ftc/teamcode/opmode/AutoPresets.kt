package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.subsystem.HuskySubsystem.SpikeLocation

class AutoPresets {

    public companion object {

        @JvmStatic
        public fun getStartPosition(alliance: Alliance, distance: Distance): Pose2d {
            val xFar = -37.0;
            val xNear = 15.0;
            val y = 62.0;

            val LD_RED_STARTPOS = Pose2d(xFar, -y, Math.toRadians(90.0))
            val SD_RED_STARTPOS = Pose2d(xNear, -y, Math.toRadians(90.0))
            val LD_BLUE_STARTPOS = Pose2d(xFar, y, Math.toRadians(270.0))
            val SD_BLUE_STARTPOS = Pose2d(xNear, y, Math.toRadians(270.0))
            return when (alliance) {
                Alliance.RED -> if (distance == Distance.SHORT) SD_RED_STARTPOS else LD_RED_STARTPOS
                Alliance.BLUE -> if (distance == Distance.SHORT) SD_BLUE_STARTPOS else LD_BLUE_STARTPOS
            }
        }

        @JvmStatic
        public fun getSpikeLocation(alliance: Alliance, distance: Distance, spikeLoc: SpikeLocation): Pose2d? {
            // hacky af
            val switchString = alliance.toString() + distance.toString() + spikeLoc.toString()
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.LEFT)) {
                return Pose2d(22.0, 39.0, Math.toRadians(270.0))
            }
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.CENTER)) {
                return Pose2d(21.0, 24.0, Math.toRadians(180.0))
            }
            if (switchString === this.name(Alliance.BLUE, Distance.SHORT, SpikeLocation.RIGHT)) {
                return Pose2d(0.0, 37.0, Math.toRadians(270.0))
            }
            if (switchString === this.name(Alliance.BLUE, Distance.LONG, SpikeLocation.LEFT)) {
                return Pose2d(-22.0, 39.0, Math.toRadians(270.0))
            }
            if (switchString === this.name(Alliance.BLUE, Distance.LONG, SpikeLocation.CENTER)) {
                return Pose2d(-42.0, 24.0, Math.toRadians(0.0))
            }
            if (switchString === this.name(Alliance.BLUE, Distance.LONG, SpikeLocation.RIGHT)) {
                return Pose2d(-46.0, 39.0, Math.toRadians(270.0))
            }

            if (switchString === this.name(Alliance.RED, Distance.SHORT, SpikeLocation.LEFT)) {
                return Pose2d(0.0, -37.0, Math.toRadians(90.0))
            }
            if (switchString === this.name(Alliance.RED, Distance.SHORT, SpikeLocation.CENTER)) {
                return Pose2d(21.0, -24.0, Math.toRadians(180.0))
            }
            if (switchString === this.name(Alliance.RED, Distance.SHORT, SpikeLocation.RIGHT)) {
                return Pose2d(22.0, -39.0, Math.toRadians(90.0))
            }
            if (switchString === this.name(Alliance.RED, Distance.LONG, SpikeLocation.LEFT)) {
                return Pose2d(-46.0, -18.0, Math.toRadians(270.0))
            }
            if (switchString === this.name(Alliance.RED, Distance.LONG, SpikeLocation.CENTER)) {
                return Pose2d(-42.0, -24.0, Math.toRadians(0.0))
            }
            if (switchString === this.name(Alliance.RED, Distance.LONG, SpikeLocation.RIGHT)) {
                return Pose2d(-32.0, -30.0, Math.toRadians(0.0))
            }
            return null
        }

        private fun name(alliance: Alliance, distance: Distance, spikeLoc: SpikeLocation): String {
            return alliance.toString() + distance.toString() + spikeLoc.toString()
        }
    }

    enum class Distance {
        SHORT, LONG
    }

    enum class Alliance {
        BLUE, RED
    }

    enum class Type {
        SPIKE, FOLLOW, BACKBOARD
    }


}
