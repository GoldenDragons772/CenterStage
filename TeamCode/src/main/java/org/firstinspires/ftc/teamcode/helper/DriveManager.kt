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
import org.firstinspires.ftc.teamcode.subsystem.subcommand.CarriageCommand
import org.firstinspires.ftc.teamcode.subsystem.subcommand.ClimbCommand
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign


class DriveManager(hardwareMap: HardwareMap, val keymap: Keymap, gamepad1: Gamepad, gamepad2: Gamepad) {

    private var precisionDrive = false;

    val drive: MecanumDriveSubsystem = MecanumDriveSubsystem(MainMecanumDrive(hardwareMap), false)
    val intake: IntakeSubsystem = IntakeSubsystem(hardwareMap)
    val linkTake: LinkTakeSubsystem = LinkTakeSubsystem(hardwareMap)
    private val bucketPivot: BucketPivotSubsystem =
        BucketPivotSubsystem(hardwareMap)
    private val dipper: DipperSubsystem =
        DipperSubsystem(hardwareMap)
    val armMotor: ArmMotorSubsystem =
        ArmMotorSubsystem(hardwareMap)
    private val drone: DroneSubsystem = DroneSubsystem(hardwareMap)
    private val gpad1: GamepadEx = GamepadEx(gamepad1)
    private val gpad2: GamepadEx = GamepadEx(gamepad2)

    init {
        CommandScheduler.getInstance().reset()
        initializeBindings()
        drive.poseEstimate = StorePos.OdoPose
    }

    fun run() {
        // Run Scheduler.
        CommandScheduler.getInstance().run()
        // Drive System
        val strafe: Double = gpad1.gamepad.right_stick_x.pow(2) * sign(gpad1.gamepad.right_stick_x.toDouble())
        val forward: Double = (gpad1.gamepad.right_stick_y.pow(2) * sign(gpad1.gamepad.right_stick_y.toDouble()))
        val spin: Double = (gpad1.gamepad.left_stick_x.pow(2) * sign(gpad1.gamepad.left_stick_x.toDouble()) * 0.90)


        val minSlowdown = .2// minimum speed is 20%
        val speedMultiplier = 1 - ((getGamepad(keymap.precisionDriveMap.second).getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) + 1)/2)*(1- minSlowdown)


        drive.drive(
            forward * speedMultiplier, strafe* speedMultiplier, spin
        )

        // Update Current Pos
        drive.update()


        val poseEstimate = drive.poseEstimate

    }

    fun setHeldBinding(button: GamepadKeys.Button, gamepad: Int, command: Command) {
        getGamepad(gamepad).getGamepadButton(button).whenHeld(command)
    }

    fun setPressedBinding(pair: Pair<GamepadKeys.Button, Int>, command: Command) {
        getGamepad(pair.second).getGamepadButton(pair.first).whenPressed(command)
    }

    private fun initializeBindings() {
        // Intake: In
        getGamepad(this.keymap.intakeMap.second).getGamepadButton(this.keymap.intakeMap.first)
            .whenPressed(InstantCommand({
                intake.runIntake()
            })).whenReleased(InstantCommand({
                    intake.stopIntake()
            }))
        // Intake: Dispense
        getGamepad(this.keymap.dispenseMap.second).getGamepadButton(this.keymap.dispenseMap.first)
            .whenHeld(InstantCommand({ intake.dispenseIntake() })).whenReleased(InstantCommand({ intake.stopIntake() }))

        // Arms: Climb
        setPressedBinding(
            this.keymap.climbMap,
            ClimbCommand(armMotor, bucketPivot, dipper, true)
        )
        // Arms: Hang
        setPressedBinding(
            this.keymap.hangMap,
            ClimbCommand(armMotor, bucketPivot, dipper, false)
        )
        // Arms: Low
        setPressedBinding(
            this.keymap.lowPositionMap,
            CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.LOW, linkTake)
        )
        // Arms: Middle
        setPressedBinding(
            this.keymap.middlePositionMap,
            CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.MIDDLE,linkTake)
        )
        // Arms: High
        setPressedBinding(
            this.keymap.highPositionMap,
            CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.HIGH,linkTake)
        )
        // Arms: Home
        setPressedBinding(
            this.keymap.homePositionMap,
            CarriageCommand(armMotor, bucketPivot, dipper, ArmMotorSubsystem.ArmPos.HOME,linkTake)
        )
        // Increment Arm Position
        setPressedBinding(
            this.keymap.incrementArmPosMap,
            InstantCommand({ armMotor.incrementArmPos() })
        )
        // Decrement Arm Position
        setPressedBinding(
            this.keymap.decrementArmPosMap,
            InstantCommand({ armMotor.decrementArmPos() })
        )
//        // Drone: Shoot
//        setPressedBinding(
//            this.keymap.shootDroneMap,
//            InstantCommand({ drone.shootDrone() })
//        )
//        // Drone: Load
//        setPressedBinding(
//            this.keymap.loadDroneMap,
//            InstantCommand({ drone.loadDrone() })
//        )

        // Precision Drive
        setPressedBinding(
                this.keymap.precisionDriveMap,
                InstantCommand({
                    precisionDrive = !precisionDrive
                })
        )

        // Increment Link Take Position
        setPressedBinding(
            this.keymap.incrementLinkTakePosMap,
            InstantCommand({ linkTake.incrementLinkTakePos() })
        )

        // Decrement Link Take Position
        setPressedBinding(
            this.keymap.decrementLinkTakePosMap,
            InstantCommand({ linkTake.decrementLinkTakePos() })
        )
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
        val precisionDriveMap: Pair<GamepadKeys.Button, Int>,
        val incrementArmPosMap: Pair<GamepadKeys.Button, Int>,
        val decrementArmPosMap: Pair<GamepadKeys.Button, Int>,
        val incrementLinkTakePosMap: Pair<GamepadKeys.Button, Int>,
        val decrementLinkTakePosMap: Pair<GamepadKeys.Button, Int>
    )


    private fun getGamepad(id: Int): GamepadEx {
        return if (id == 1) {
            gpad1
        } else {
            gpad2
        }
    }

    private fun enableManualControl() {
        armMotor.setArmMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    private fun disableManualControl(id: Int) {
        armMotor.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        getGamepad(id).gamepad.rumble(1000)
    }

}