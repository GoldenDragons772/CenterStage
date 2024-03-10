package org.firstinspires.ftc.teamcode.helper

import com.acmerobotics.roadrunner.geometry.Pose2d
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

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
        Pose2d(60.0,  35.0),
        Pose2d(60.0, -35.0)
    );

    /**
     * The value added and subtracted from the exit direction to create the upper and lower bounds.
     */
    private val exitBound: Float = 25.0f;

    /**
     * The maximum distance to the backdrop wherein the damper will apply.
     */
    private val radius: Float = 5.0f;

    /**
     * The factor by which to damp when all requirements are met.
     */
    private val factor: Float = .75f;

    /**
     * The main function that outputs the speed at which the robot should move, factoring in all calculations etc.
     */
    fun damp(currentPosition: Pose2d, xSpeed: Float, ySpeed: Float): MutableList<Float>? {
        if (!enabled) return mutableListOf(xSpeed, ySpeed)
        if (lastPosition == null) {
            lastPosition = currentPosition
            return null
        }
        if (!inRadius(currentPosition)) return null
        val direction = calculateDirection(currentPosition)
        if (canExit(direction)) return null
        return mutableListOf(xSpeed*factor, ySpeed*factor)
    }

    /**
     * Determines if the direction the robot is moving in is within the exit sector.
     */
    private fun canExit(direction: Double): Boolean {
        val oppositeDirection = direction + PI
        return oppositeDirection - exitBound < direction && direction < oppositeDirection + exitBound
    }

    /**
     * Determines if the robot is within the radius of a backboard.
     */
    private fun inRadius(currentPosition: Pose2d): Boolean {
        for (i in backboardPositions) {
            if (distance(currentPosition, i) > radius) continue
            return true
        }
        return false
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