package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import kotlin.Pair;
import org.firstinspires.ftc.teamcode.helper.DriveManager;
import org.firstinspires.ftc.teamcode.subsystem.LinkTakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.TrajectoryFollowerCommand;

@TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {

    TrajectoryFollowerCommand driveToBackDropBlue;
    DriveManager driveManager;

    @Override
    public void initialize() {
        DriveManager.Keymap keymap = new DriveManager.Keymap(
                new Pair<>(GamepadKeys.Button.RIGHT_BUMPER, 1), // Run intake
                new Pair<>(GamepadKeys.Button.LEFT_BUMPER, 1), // Run dispense
                new Pair<>(GamepadKeys.Button.DPAD_UP, 2),     // Climb
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 2),   // Hang
                new Pair<>(GamepadKeys.Button.DPAD_RIGHT, 1),  // Middle Position
                new Pair<>(GamepadKeys.Button.DPAD_LEFT, 1),    // Low Position
                new Pair<>(GamepadKeys.Button.DPAD_UP, 1),     // Top Position
                new Pair<>(GamepadKeys.Button.DPAD_DOWN, 1),   // Home Position
                new Pair<>(GamepadKeys.Button.RIGHT_STICK_BUTTON, 1), // Precision Drive
                new Pair<>(GamepadKeys.Button.Y, 1),           // Increment ArmPos
                new Pair<>(GamepadKeys.Button.A, 1),            // Decrement ArmPos
                new Pair<>(GamepadKeys.Button.B, 1),            // Increment LinkTakePos
                new Pair<>(GamepadKeys.Button.X, 1)            // Decrement LinkTakePos
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

        // Manual Arm Control...

//        // Increment ArmPos
//        if(gamepad1.right_trigger > 0.1) {
//            driveManager.getArmMotor().setArmManualControl((int) ((driveManager.getArmMotor().getAvgArmPosition()) + (100 * gamepad1.right_trigger)));
//        }
//
//        // Decrease ArmPos
//        if(gamepad1.left_trigger < 0.1) {
//            driveManager.getArmMotor().setArmManualControl((int) ((driveManager.getArmMotor().getAvgArmPosition()) - (100 * gamepad1.left_trigger)));
//        }

//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//        telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
//        telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
//        telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
//        telemetry.addData("Correction", armMotor.correction);
////            telemetry.addData("DipperRightServo", dipper.rightDipperServo.getPosition());
////            telemetry.addData("DipperLeftServo", dipper.leftDipperServo.getPosition());
//        telemetry.update();


        // Control Intake via Triggers
        if(!gamepad1.left_bumper) {
            if(!gamepad1.right_bumper) {
                if(gamepad1.right_trigger > 0.1) {
                    // Square the input for better control
                    double intakePosition = Math.max(0.3, Math.min(0.75, Math.pow(gamepad1.right_trigger, 2) * 0.75));
                    driveManager.getLinkTake().setLinkTakePosRaw(intakePosition);
                    driveManager.getIntake().runIntake();
                    telemetry.addData("LinkTake Position", intakePosition);
                } else {
                    driveManager.getLinkTake().setLinkTakePos(LinkTakeSubsystem.linkPos);
                    driveManager.getIntake().stopIntake();
                }
            }
        }
        telemetry.update();
    }
}
