package org.firstinspires.ftc.teamcode.helper

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline

class TrajectoryManager {

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
        public fun getSpikeLocation(alliance: Alliance, distance: Distance, spikeLoc: PropDetectionPipeline.propPos): Pose2d? {
            // hacky af
            val switchString = name(alliance, distance, spikeLoc)
            val poseMap = mapOf(
                    name(Alliance.BLUE, Distance.SHORT, PropDetectionPipeline.propPos.LEFT) to Pose2d(23.0, 39.0, Math.toRadians(270.0)),
                    name(Alliance.BLUE, Distance.SHORT, PropDetectionPipeline.propPos.CENTER) to Pose2d(21.0, 25.0, Math.toRadians(180.0)),
                    name(Alliance.BLUE, Distance.SHORT, PropDetectionPipeline.propPos.RIGHT) to Pose2d(10.0, 37.0, Math.toRadians(200.0)),
                    name(Alliance.BLUE, Distance.LONG, PropDetectionPipeline.propPos.LEFT) to Pose2d(-32.0, 37.0, Math.toRadians(320.0)),
                    name(Alliance.BLUE, Distance.LONG, PropDetectionPipeline.propPos.CENTER) to Pose2d(-42.0, 24.0, Math.toRadians(0.0)),
                    name(Alliance.BLUE, Distance.LONG, PropDetectionPipeline.propPos.RIGHT) to Pose2d(-44.0, 18.0, Math.toRadians(90.0)),
                    name(Alliance.RED, Distance.SHORT, PropDetectionPipeline.propPos.LEFT) to Pose2d(10.0, -37.0, Math.toRadians(155.0)),
                    name(Alliance.RED, Distance.SHORT, PropDetectionPipeline.propPos.CENTER) to Pose2d(21.0, -24.0, Math.toRadians(180.0)),
                    name(Alliance.RED, Distance.SHORT, PropDetectionPipeline.propPos.RIGHT) to Pose2d(23.0, -39.0, Math.toRadians(90.0)),
                    name(Alliance.RED, Distance.LONG, PropDetectionPipeline.propPos.LEFT) to Pose2d(-44.0, -18.0, Math.toRadians(270.0)),
                    name(Alliance.RED, Distance.LONG, PropDetectionPipeline.propPos.CENTER) to Pose2d(-42.0, -25.0, Math.toRadians(0.0)),
                    name(Alliance.RED, Distance.LONG, PropDetectionPipeline.propPos.RIGHT) to Pose2d(-32.0, -37.0, Math.toRadians(30.0))
            )
            return poseMap[switchString] ?: Pose2d(22.0, -39.0, Math.toRadians(90.0)) // Default
        }

        private fun name(alliance: Alliance, distance: Distance, spikeLoc: PropDetectionPipeline.propPos): String {
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
        SPIKE, FOLLOW, BACKBOARD, CARRIER, PARK
    }


}
