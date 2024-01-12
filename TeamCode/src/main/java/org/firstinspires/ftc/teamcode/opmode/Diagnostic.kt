package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem

@Autonomous(name = "Diagnostic")
class Diagnostic : LinearOpMode() {
    private lateinit var mecDrive: MainMecanumDrive;
    private lateinit var armMotorSubsystem: ArmMotorSubsystem
    private lateinit var dipperSubsystem: DipperSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    override fun runOpMode() {
        mecDrive = MainMecanumDrive(hardwareMap)
        armMotorSubsystem = ArmMotorSubsystem(hardwareMap)
        dipperSubsystem = DipperSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(hardwareMap)
        waitForStart()
        testCircleDrive()
        testSquareStrafe()
        testRotation()
        testSlideExtend()
        testDipperRotate()
        testSlideRetract()
        testIntake()

    }


    private fun testCircleDrive() {
        val traj = mecDrive.trajectoryBuilder(Pose2d())
            .splineTo(Vector2d(15.0, 15.0), Math.toRadians(90.0))
            .splineTo(Vector2d(0.0, 30.0), Math.toRadians(180.0))
            .splineTo(Vector2d(-15.0, 15.0), Math.toRadians(270.0))
            .splineTo(Vector2d(0.0, 0.0), 0.0).build()

        this.telemetry.addLine("TEST: Circle Drive")
        mecDrive.followTrajectory(traj)
        while (mecDrive.isBusy) {
            sleep(20)
        }
        this.telemetry.addLine("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testSquareStrafe() {
        val traj = mecDrive.trajectoryBuilder(Pose2d())
            .lineTo(Vector2d(15.0, 15.0))
            .lineTo(Vector2d(0.0, 30.0))
            .lineTo(Vector2d(-15.0, 15.0))
            .lineTo(Vector2d(0.0, 0.0))
            .build()

        this.telemetry.addLine("TEST: Square Strafe")
        mecDrive.followTrajectory(traj)
        while (mecDrive.isBusy) {
            sleep(20)
        }
        this.telemetry.addLine("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testRotation() {

        this.telemetry.addLine("TEST: Spin (1080deg)")
        mecDrive.turn(Math.toRadians(360.0 * 3.0))
        this.telemetry.addLine("End Pos: (${this.mecDrive.poseEstimate.x}, ${this.mecDrive.poseEstimate.y})/${this.mecDrive.poseEstimate.heading}")
    }

    private fun testSlideExtend() {
        this.telemetry.addLine("TEST: Slide Extend (2100T)")
        armMotorSubsystem.setArmToPos(2100)
        this.telemetry.addLine("Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
    }

    private fun testDipperRotate() {
        this.telemetry.addLine("TEST: Dipper Rotation (Scoring)")
        this.dipperSubsystem.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION)
        this.telemetry.addLine("Pos (L): ${this.dipperSubsystem.leftDipperServo.position} Pos(R): ${this.dipperSubsystem.rightDipperServo.position} ")
    }

    private fun testSlideRetract() {
        this.telemetry.addLine("TEST: Slide Retract (0T)")
        armMotorSubsystem.setArmToPos(0)
        this.telemetry.addLine("Pos (L): ${armMotorSubsystem.leftArmMotor.currentPosition} Pos (R): ${armMotorSubsystem.rightArmMotor.currentPosition}")
    }

    private fun testIntake() {
        this.telemetry.addLine("TEST: Intake")
        intakeSubsystem.runIntake()
        sleep(1000)
        intakeSubsystem.stopIntake()
        sleep(1000)
        intakeSubsystem.dispenseIntake()
        sleep(1000)
        this.telemetry.addLine("Intake test complete.")
    }
}
