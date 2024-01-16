package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem
import kotlin.math.exp
import kotlin.math.sqrt

@Autonomous(name = "Diagnostic")
class Diagnostic : LinearOpMode() {
    private lateinit var mecDrive: MainMecanumDrive;
    private lateinit var armMotorSubsystem: ArmMotorSubsystem
    private lateinit var dipperSubsystem: DipperSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private val strafeLength = 3.0;
    override fun runOpMode() {
        mecDrive = MainMecanumDrive(hardwareMap)
        armMotorSubsystem = ArmMotorSubsystem(hardwareMap)
        dipperSubsystem = DipperSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(hardwareMap)
        waitForStart()
        this.telemetry.addLine("Creating github issues...")
        this.telemetry.addLine("Decreasing the number of servos...")
        this.telemetry.addLine("Fixing \"programming\" issues...")
        testCircleDrive()
        testSquareStrafe()
        testRotation()
        testSlideExtend()
        testDipperRotate()
        testSlideRetract()
        testIntake()
        // Filibuster
        while (true) {
            sleep(1000)
        }
    }


    private fun testCircleDrive() {
        this.telemetry.addLine("TEST: Circle Drive (0.0, 0.0)")
        mecDrive.followTrajectory(
            mecDrive.trajectoryBuilder(Pose2d())
                .splineTo(Vector2d(strafeLength, strafeLength), Math.toRadians(90.0))
                .splineTo(Vector2d(0.0, 2 * strafeLength), Math.toRadians(180.0))
                .splineTo(Vector2d(-strafeLength, strafeLength), Math.toRadians(270.0))
                .splineTo(Vector2d(0.0, 0.0), 0.0).build()
        )
        while (mecDrive.isBusy) {
            sleep(20)
        }
        this.telemetry.addLine("Err: ${sqrt(exp(this.mecDrive.poseEstimate.x)-exp(this.mecDrive.poseEstimate.y))} End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testSquareStrafe() {
        this.telemetry.addLine("TEST: Square Strafe (0.0, 0.0)")
        mecDrive.followTrajectorySequence(
            mecDrive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
                .forward(strafeLength)
                .strafeLeft(strafeLength)
                .back(strafeLength)
                .strafeRight(strafeLength)
                .build()
        )
        while (mecDrive.isBusy) {
            sleep(20)
        }
        this.telemetry.addLine("Err: ${sqrt(exp(this.mecDrive.poseEstimate.x)-exp(this.mecDrive.poseEstimate.y))} End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testRotation() {
        this.telemetry.addLine("TEST: Spin (1080deg)")
        val originalHeading = mecDrive.externalHeading
        mecDrive.turn(Math.toRadians(360.0 * 3.0))
        this.telemetry.addLine("Err: ${mecDrive.externalHeading - originalHeading} End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testSlideExtend() {
        this.telemetry.addLine("TEST: Slide Extend (2200T)")
        armMotorSubsystem.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH)
        armMotorSubsystem.waitForIdle()

        this.telemetry.addLine("Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
    }

    private fun testDipperRotate() {
        this.telemetry.addLine("TEST: Dipper Rotation (Scoring)")
        // This assumes that the dipper is already in the loading position.
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION)
        this.dipperSubsystem.waitForIdle()
        this.telemetry.addLine("Pos (L): ${this.dipperSubsystem.leftPosition} Pos(R): ${this.dipperSubsystem.rightPosition} ")
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION)
        this.dipperSubsystem.waitForIdle()
        this.telemetry.addLine("Pos (L): ${this.dipperSubsystem.leftPosition} Pos(R): ${this.dipperSubsystem.rightPosition} ")
    }

    private fun testSlideRetract() {
        this.telemetry.addLine("TEST: Slide Retract (0T)")
        armMotorSubsystem.setArmToPos(ArmMotorSubsystem.ArmPos.HOME)
        this.telemetry.addLine("Err:${armMotorSubsystem.avgArmPosition} Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
    }

    private fun testIntake() {
        this.telemetry.addLine("TEST: Intake")
        intakeSubsystem.runIntake()
        sleep(1000)
        intakeSubsystem.stopIntake()
        sleep(1000)
        intakeSubsystem.dispenseIntake()
        sleep(1000)
        intakeSubsystem.stopIntake()
        this.telemetry.addLine("Intake test complete.")
        this.telemetry.update()
    }
}
