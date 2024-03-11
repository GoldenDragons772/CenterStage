package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import kotlin.Pair;
import org.firstinspires.ftc.teamcode.helper.DriveManager;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;


@TeleOp(name = "Duo", group = "TeleOp")
public class Duo extends CommandOpMode {

//    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
    private DriveManager driveManager;

    private MecanumDriveSubsystem drive;

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
                new Pair<>(GamepadKeys.Button.X, 1),            // Load,
                new Pair<>(GamepadKeys.Button.RIGHT_BUMPER, 1) // Precision Drive
        );
        driveManager = new DriveManager(hardwareMap, keymap, gamepad1, gamepad2);

        drive = driveManager.getDrive();
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

        // Control Intake Via Triggers


        if(!gamepad2.left_bumper) {
            if(!gamepad2.right_bumper) {
                if(gamepad2.right_trigger > 0.1) {
                    // Square the input for better control
                    double intakePower = Math.pow(gamepad2.right_trigger, 2);
                    driveManager.getLinkTake().setArmPosRaw(intakePower * 0.75);
                    driveManager.getIntake().runIntake();
                } else {
                    driveManager.getLinkTake().setArmPosRaw(0.2);
                    driveManager.getIntake().stopIntake();
                }
            }
        }
    }
}
