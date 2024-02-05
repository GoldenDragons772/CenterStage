package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import kotlin.Pair;
import org.firstinspires.ftc.teamcode.helper.DriveManager;

@TeleOp(name = "Duo", group = "TeleOp")
public class Duo extends CommandOpMode {

//    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private DriveManager driveManager;

    @Override
    public void initialize() {

        DriveManager.Keymap keymap = new DriveManager.Keymap(
                new Pair<>(GamepadKeys.Button.RIGHT_BUMPER, 2), // Run intake
                new Pair<>(GamepadKeys.Button.LEFT_BUMPER, 2), // Run dispense
                new Pair<>(GamepadKeys.Button.DPAD_UP, 1),     // Climb
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 1),   // Hang
                new Pair<>(GamepadKeys.Button.DPAD_RIGHT, 2),  // Middle Position
                new Pair<>(GamepadKeys.Button.DPAD_LEFT, 2),    // Low Position
                new Pair<>(GamepadKeys.Button.DPAD_UP, 2),     // Top Position
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 2),   // Home Position
                new Pair<>(GamepadKeys.Button.X, 2),           // Arm Manual Control
                new Pair<>(GamepadKeys.Button.B, 2),           // Arm Reset
                new Pair<>(GamepadKeys.Button.Y, 1),           // Shoot
                new Pair<>(GamepadKeys.Button.X, 1)            // Load
        );
        driveManager = new DriveManager(hardwareMap, keymap, gamepad1, gamepad2);
    }

    @Override
    public void run() {
        driveManager.run();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//        telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
//        telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
//        telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
//        telemetry.addData("Correction", armMotor.correction);
//        telemetry.addData("ManualArmPower", armPower);
//        telemetry.update();

    }
}
