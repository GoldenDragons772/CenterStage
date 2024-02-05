package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem
import kotlin.math.exp
import kotlin.math.sqrt

@TeleOp(name = "Diagnostic")
class Diagnostic : LinearOpMode() {
    private lateinit var mecDrive: MainMecanumDrive
    private lateinit var armMotorSubsystem: ArmMotorSubsystem
    private lateinit var dipperSubsystem: DipperSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var ftcDashboard: FtcDashboard
    private lateinit var bucketPivotSubsystem: BucketPivotSubsystem
    private val strafeLength = 3.0
    override fun runOpMode() {
        ftcDashboard = FtcDashboard.getInstance()
        mecDrive = MainMecanumDrive(hardwareMap)
        armMotorSubsystem = ArmMotorSubsystem(hardwareMap)
        dipperSubsystem = DipperSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(hardwareMap)
        bucketPivotSubsystem = BucketPivotSubsystem(hardwareMap)
        CommandScheduler.getInstance().reset()
        val gpad = GamepadEx(gamepad1)
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
        var lastTest = 0
        gpad.getGamepadButton(GamepadKeys.Button.A).whenPressed(Runnable {
            tests[lastTest]()
            lastTest++
        })
        // Filibuster
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run()
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
        this.bucketPivotSubsystem.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS)
        telemetryStrings += ("Pos (L): ${this.dipperSubsystem.leftPosition} Pos(R): ${this.dipperSubsystem.rightPosition} ")
        sendTelemetryPacket(telemetryStrings)
    }

    private fun testDipperReturn() {
        val telemetryStrings = mutableListOf<String>()
        telemetryStrings += ("TEST: Dipper Rotation (Loading)")
        // This assumes that the dipper is already in the scoring position.
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION)
        this.bucketPivotSubsystem.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS)
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