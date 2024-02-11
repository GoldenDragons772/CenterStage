package org.firstinspires.ftc.teamcode.helper

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive
import org.firstinspires.ftc.teamcode.subsystem.*
import kotlin.math.pow
import kotlin.math.sign


class DriveManager(hardwareMap: HardwareMap, val keymap: Keymap, gamepad1: Gamepad, gamepad2: Gamepad) {

    val drive: MecanumDriveSubsystem = MecanumDriveSubsystem(MainMecanumDrive(hardwareMap), false)
    private val intake: IntakeSubsystem = IntakeSubsystem(hardwareMap)
    private val bucket: BucketSubsystem = BucketSubsystem(hardwareMap)
    private val bucketPivot: BucketPivotSubsystem = BucketPivotSubsystem(hardwareMap)
    private val dipper: DipperSubsystem = DipperSubsystem(hardwareMap)
    private val armMotor: ArmMotorSubsystem = ArmMotorSubsystem(hardwareMap)
    private val drone: DroneSubsystem = DroneSubsystem(hardwareMap)
    private val gpad1: GamepadEx = GamepadEx(gamepad1)
    private val gpad2: GamepadEx = GamepadEx(gamepad2)

    init {
        CommandScheduler.getInstance().reset();
        initializeBindings()
        drive.poseEstimate = StorePos.OdoPose
    }

    fun run() {
        // Run Scheduler.
        CommandScheduler.getInstance().run()

        // Drive System
        val strafe: Double = gpad1.gamepad.right_stick_x.pow(2) * sign(gpad1.gamepad.right_stick_x.toDouble())
        val forward: Double = gpad1.gamepad.right_stick_y.pow(2) * sign(gpad1.gamepad.right_stick_y.toDouble())
        val spin: Double = gpad1.gamepad.left_stick_x.pow(2) * sign(gpad1.gamepad.left_stick_x.toDouble())

        val speedMultiplier = 1.0

        drive.drive(
            forward * speedMultiplier, strafe * speedMultiplier, spin * speedMultiplier
        )

//        // Manual Arm Control (Make sure to Un-Lock First)
//        val armPower = (gpad2.gamepad.right_trigger - gpad2.gamepad.left_trigger).toDouble()
//        if (armPower > 0.1 || armPower < -0.1) {
//            armMotor.setArmPower(armPower)
//        }

        drive.update()

        val poseEstimate = drive.poseEstimate

    }

    public fun setHeldBinding(button: GamepadKeys.Button, gamepad: Int, command: Command) {
        getGamepad(gamepad).getGamepadButton(button).whenHeld(command)
    }

    private fun initializeBindings() {
        // Intake: In
        getGamepad(this.keymap.intakeMap.second).getGamepadButton(this.keymap.intakeMap.first)
            .whenHeld(InstantCommand({ runIntake() })).whenReleased(InstantCommand({ stopIntake() }));
        // Intake: Dispense
        getGamepad(this.keymap.dispenseMap.second).getGamepadButton(this.keymap.dispenseMap.first)
            .whenHeld(InstantCommand({ dispense() })).whenReleased(InstantCommand({ stopDispense() }));
        // Arms: Climb
        getGamepad(this.keymap.climbMap.second).getGamepadButton(this.keymap.climbMap.first)
            .whenHeld(InstantCommand({ climb() }));
        // Arms: Hang
        getGamepad(this.keymap.hangMap.second).getGamepadButton(this.keymap.hangMap.first)
            .whenHeld(InstantCommand({ hang() }));
        // Arms: Low
        getGamepad(this.keymap.lowPositionMap.second).getGamepadButton(this.keymap.lowPositionMap.first)
            .whenHeld(InstantCommand({ lowPosition() }));
        // Arms: Middle
        getGamepad(this.keymap.middlePositionMap.second).getGamepadButton(this.keymap.middlePositionMap.first)
            .whenHeld(InstantCommand({ middlePosition() }));
        // Arms: High
        getGamepad(this.keymap.highPositionMap.second).getGamepadButton(this.keymap.highPositionMap.first)
            .whenHeld(InstantCommand({ topPosition() }));
        // Arms: High
        getGamepad(this.keymap.homePositionMap.second).getGamepadButton(this.keymap.homePositionMap.first)
            .whenHeld(InstantCommand({ homePosition() }));
        // Arm Manual Control: Enable
        getGamepad(this.keymap.enableManualControlMap.second).getGamepadButton(this.keymap.enableManualControlMap.first)
            .whenHeld(InstantCommand({ enableManualControl() }))
        // Arm Manual Control: Disable
        getGamepad(this.keymap.disableManualControlMap.second).getGamepadButton(this.keymap.disableManualControlMap.first)
            .whenHeld(InstantCommand({ disableManualControl(this.keymap.disableManualControlMap.second) }))
        // Drone: Shoot
        getGamepad(this.keymap.shootDroneMap.second).getGamepadButton(this.keymap.shootDroneMap.first)
            .whenHeld(InstantCommand({ shootDrone() }))
        // Drone: Load
        getGamepad(this.keymap.loadDroneMap.second).getGamepadButton(this.keymap.loadDroneMap.first)
            .whenHeld(InstantCommand({ loadDrone() }))

        getGamepad(this.keymap.orientMap.second).getGamepadButton(this.keymap.orientMap.first)
            .whenHeld(InstantCommand({ drive.orient() }))

    }

    // Keymap. values are given as a pair of the button and the gamepad number.
    class Keymap(
        val intakeMap: Pair<GamepadKeys.Button, Int>,
        val dispenseMap: Pair<GamepadKeys.Button, Int>,
        val climbMap: Pair<GamepadKeys.Button, Int>,
        val hangMap: Pair<GamepadKeys.Button, Int>,
        val middlePositionMap: Pair<GamepadKeys.Button, Int>,
        val lowPositionMap: Pair<GamepadKeys.Button, Int>,
        val highPositionMap: Pair<GamepadKeys.Button, Int>,
        val homePositionMap: Pair<GamepadKeys.Button, Int>,
        val enableManualControlMap: Pair<GamepadKeys.Button, Int>,
        val disableManualControlMap: Pair<GamepadKeys.Button, Int>,
        val shootDroneMap: Pair<GamepadKeys.Button, Int>,
        val loadDroneMap: Pair<GamepadKeys.Button, Int>,
        val orientMap: Pair<GamepadKeys.Button, Int>
    )


    private fun getGamepad(id: Int): GamepadEx {
        return if (id == 1) {
            gpad1
        } else {
            gpad2
        }
    }

    private fun runIntake() {
        if (armMotor.armPos != ArmMotorSubsystem.ArmPos.HOME) return
        intake.runIntake();
        bucket.intakePixels();
    }

    private fun stopIntake() {
        intake.stopIntake();
        bucket.stopBucket();
    }

    private fun dispense() {
        bucket.dispensePixels()
        intake.dispenseIntake()
    }

    private fun stopDispense() {
        bucket.stopBucket()
        intake.stopIntake()
    }

    private fun climb() {
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
    }

    private fun hang() {
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
        armMotor.setArmToPos(100);
    }

    private fun middlePosition() {
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.MIDDLE);
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
    }

    private fun lowPosition() {
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.LOW);
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
    }

    private fun topPosition() {
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HIGH);
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
    }

    private fun homePosition() {
        armMotor.setArmToPos(ArmMotorSubsystem.ArmPos.HOME);
        dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
        bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
    }

    private fun enableManualControl() {
        armMotor.setArmMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private fun disableManualControl(id: Int) {
        armMotor.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getGamepad(id).gamepad.rumble(1000);
    }

    private fun shootDrone() {
        drone.shootDrone();
    }

    private fun loadDrone() {
        drone.loadDrone();
    }

}