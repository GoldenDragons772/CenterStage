package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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
    private lateinit var ftcDashboard: FtcDashboard
    private val strafeLength = 3.0;
    override fun runOpMode() {
        ftcDashboard = FtcDashboard.getInstance()
        mecDrive = MainMecanumDrive(hardwareMap)
        armMotorSubsystem = ArmMotorSubsystem(hardwareMap)
        dipperSubsystem = DipperSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(hardwareMap)
        waitForStart()

        sendTelemetryPacket(
            mutableListOf(
                "Creating github issues...", "Decreasing the number of servos...", "Fixing \"programming\" issues..."
            )
        )
        val tests = mutableListOf({ testCircleDrive() },
            { testSquareStrafe() },
            { testRotation() },
            { testSlideExtend() },
            { testDipperLeave() },
            { testDipperReturn() },
            { testSlideRetract() },
            { testIntake() })
        for (i in tests) {
            i()
            waitForConfirmation()
        }
        // Filibuster
        while (true) {
            sleep(1000)
        }
    }


    private fun waitForConfirmation() {
        while (!gamepad1.a) {
            sleep(10)
        }
    }


    private fun testCircleDrive() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += "TEST: Circle Drive (0.0, 0.0)"
        mecDrive.followTrajectory(
            mecDrive.trajectoryBuilder(Pose2d()).splineTo(Vector2d(strafeLength, strafeLength), Math.toRadians(90.0))
                .splineTo(Vector2d(0.0, 2 * strafeLength), Math.toRadians(180.0))
                .splineTo(Vector2d(-strafeLength, strafeLength), Math.toRadians(270.0))
                .splineTo(Vector2d(0.0, 0.0), 0.0).build()
        )
        while (mecDrive.isBusy) {
            sleep(20)
        }
        telemetryStrings += ("Err: ${sqrt(exp(this.mecDrive.poseEstimate.x) - exp(this.mecDrive.poseEstimate.y))}")
        telemetryStrings += ("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testSquareStrafe() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Square Strafe (0.0, 0.0)")
        mecDrive.followTrajectorySequence(
            mecDrive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0)).forward(strafeLength).strafeLeft(strafeLength)
                .back(strafeLength).strafeRight(strafeLength).build()
        )
        while (mecDrive.isBusy) {
            sleep(20)
        }
        telemetryStrings += ("Err: ${sqrt(exp(this.mecDrive.poseEstimate.x) - exp(this.mecDrive.poseEstimate.y))}")
        telemetryStrings += ("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")

        sendTelemetryPacket(telemetryStrings)
    }

    private fun testRotation() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Spin (1080deg)")
        val originalHeading = mecDrive.externalHeading
        mecDrive.turn(Math.toRadians(360.0 * 3.0))
        telemetryStrings += ("Err: ${mecDrive.externalHeading - originalHeading}")
        telemetryStrings += ("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testSlideExtend() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Slide Extend (2200T)")
        armMotorSubsystem.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH)
        armMotorSubsystem.waitForIdle()

        telemetryStrings += ("Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testDipperLeave() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Dipper Rotation (Scoring)")
        // This assumes that the dipper is already in the loading position.
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION)
        telemetryStrings += ("Pos (L): ${this.dipperSubsystem.leftPosition} Pos(R): ${this.dipperSubsystem.rightPosition} ")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testDipperReturn() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Dipper Rotation (Loading)")
        // This assumes that the dipper is already in the scoring position.
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION)
        telemetryStrings += ("Pos (L): ${this.dipperSubsystem.leftPosition} Pos(R): ${this.dipperSubsystem.rightPosition} ")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testSlideRetract() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Slide Retract (0T)")
        armMotorSubsystem.setArmToPos(ArmMotorSubsystem.ArmPos.HOME)
        telemetryStrings += ("Err:${armMotorSubsystem.avgArmPosition}")
        telemetryStrings += ("Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testIntake() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Intake")
        intakeSubsystem.runIntake()
        sleep(1000)
        intakeSubsystem.stopIntake()
        sleep(1000)
        intakeSubsystem.dispenseIntake()
        sleep(1000)
        intakeSubsystem.stopIntake()
        telemetryStrings += ("Intake test complete.")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun sendTelemetryPacket(strings: MutableList<String>) {
        val packet = TelemetryPacket()
        strings.forEach { packet.addLine(it + "\n"); this.telemetry.addLine(it) }
        ftcDashboard.sendTelemetryPacket(packet)
        this.telemetry.update()

    }
}