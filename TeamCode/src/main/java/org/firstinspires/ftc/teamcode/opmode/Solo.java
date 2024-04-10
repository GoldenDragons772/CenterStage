package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import kotlin.Pair;
import org.firstinspires.ftc.teamcode.helper.DriveManager;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;

@TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {

//    TrajectoryFollowerCommand driveToBackDropBlue;
    DriveManager driveManager;

    @Override
    public void initialize() {
        DriveManager.Keymap keymap =    new DriveManager.Keymap(
                new Pair<>(GamepadKeys.Button.RIGHT_BUMPER, 1), // Run intake
                new Pair<>(GamepadKeys.Button.LEFT_BUMPER, 1), // Run dispense
                new Pair<>(GamepadKeys.Button.DPAD_UP, 2),     // Climb
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 2),   // Hang
                new Pair<>(GamepadKeys.Button.DPAD_RIGHT, 1),  // Middle Position
                new Pair<>(GamepadKeys.Button.DPAD_LEFT, 1),    // Low Position
                new Pair<>(GamepadKeys.Button.DPAD_UP, 1),     // Top Position
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 1),   // Home Position
                new Pair<>(null, 1), // Precision Drive
                new Pair<>(GamepadKeys.Button.Y, 1),           // Increment ArmPos
                new Pair<>(GamepadKeys.Button.A, 1),            // Decrement ArmPos
                new Pair<>(GamepadKeys.Button.B, 1),            // Increment LinkTakePos
                new Pair<>(GamepadKeys.Button.X, 1),            // Decrement LinkTakePos
                new Pair<>(GamepadKeys.Button.Y, 2),            // Shoot Drone
                new Pair<>(GamepadKeys.Button.A, 2),            // Load Drone
                new Pair<>(null, 1)      // Run LinkTake
        );


        driveManager = new DriveManager(hardwareMap, keymap, gamepad1, gamepad2);

        // Auto Drive Cmds
//        Trajectory BACKDROP_BLUE_LEFT = driveManager.getDrive().trajectoryBuilder(driveManager.getDrive().getPoseEstimate())
//                .lineToLinearHeading(new Pose2d(54, 41, Math.toRadians(180)))
//                .build();
//
//        driveToBackDropBlue = new TrajectoryFollowerCommand(driveManager.getDrive(), BACKDROP_BLUE_LEFT);
//
//        // Auto Align Feature
//        driveManager.setBinding(GamepadKeys.Button.X, 1, driveToBackDropBlue);
    }

    @Override
    public void run() {
        driveManager.run();

        telemetry.addData("ArmPos", ArmMotorSubsystem.armPos.name());

        telemetry.update();
    }
}
