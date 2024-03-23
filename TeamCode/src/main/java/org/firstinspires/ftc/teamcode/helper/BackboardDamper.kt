package org.firstinspires.ftc.teamcode.helper

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.*

class BackboardDamper {
    /**
     * Whether the damper is enabled or not.
     */
    private var enabled = true;

    /**
     * The position at the last iteration.
     */
    private var lastPosition: Pose2d? = null;

    /**
     *  The position of the backboard.
     */
    private val backboardPositions: MutableList<Pose2d> = mutableListOf(
        Pose2d(59.0,  35.0),
        Pose2d(59.0, -35.0)
    );

    /**
     * The value added and subtracted from the exit direction to create the upper and lower bounds.
     */
    private val exitBound: Double = Math.toRadians(70.0);

   /**
     * The maximum distance to the backdrop wherein the damper will apply.
     */
    private val radius: Float = 30.0f;

    /**
     * The factor by which to damp when all requirements are met.
     */
    private val factor: Float = .25f;

    /**
     * The main function that outputs the speed at which the robot should move, factoring in all calculations etc.
     */
    fun damp(currentPosition: Pose2d, xSpeed: Double, ySpeed: Double): MutableList<Double>? {
        if (!enabled) return mutableListOf(xSpeed, ySpeed)
        if (lastPosition == null) {
            lastPosition = Pose2d(-37.0,62.0,Math.toRadians(270.0))
            return null
        }
        val direction = calculateDirection(currentPosition)
        val packet = TelemetryPacket()
        packetAddDebugDirectionLine(currentPosition, direction, packet)
        for (i in backboardPositions){
            packet.fieldOverlay().fillCircle(i.x, i.y, 5.0)
        }

        val closestBackboard = inRadius(currentPosition)
        if (closestBackboard == null){
            lastPosition = currentPosition
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            return null
        }


        if (canExit(direction, currentPosition, closestBackboard)) {
            lastPosition = currentPosition
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            return null
        }
        lastPosition = currentPosition
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
        return mutableListOf(xSpeed*factor, ySpeed*factor)
    }
    private fun packetAddDebugDirectionLine(currentPosition: Pose2d, direction: Double, packet: TelemetryPacket){
        packet.fieldOverlay().strokeLine(currentPosition.x, currentPosition.y, currentPosition.x + cos(direction)*20, currentPosition.y + sin(direction)*20)
    }

    /**
     * Determines if the direction the robot is moving in is within the exit sector.
     */
    private fun canExit(direction: Double, currentPosition: Pose2d, closestBackboard: Pose2d): Boolean {
        val oppositeDirection = atan2((closestBackboard.y - currentPosition.y), (closestBackboard.x - currentPosition.x)) % (2* PI)
        val remainderDirection = direction % (2*PI)
        return !(oppositeDirection - exitBound < remainderDirection && remainderDirection < oppositeDirection + exitBound)
    }

    /**
     * Determines if the robot is within the radius of a backboard.
     */
    private fun inRadius(currentPosition: Pose2d): Pose2d? {
        for (i in backboardPositions) {
            if (distance(currentPosition, i) > radius) continue
            return i
        }
        return null
    }

    /**
     * Calculates the direction in which the robot is moving based on angle from the last position
     */
    private fun calculateDirection(currentPosition: Pose2d): Double {
        return atan2((currentPosition.y - lastPosition!!.y), (currentPosition.x - lastPosition!!.x))
    }

    /**
     * Basic distance calculation using the pythagorean theorem.
     */
    fun distance(p1: Pose2d, p2: Pose2d): Double {
        return sqrt((p1.x - p2.x).pow(2) + (p1.y - p2.y).pow(2))
    }
}