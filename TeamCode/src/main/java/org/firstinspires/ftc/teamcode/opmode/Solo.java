package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DipperSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;

@TeleOp(name = "Solo", group = "TeleOp")
public class Solo extends CommandOpMode {

    private MecanumDriveSubsystem drive;

    private IntakeSubsystem intake;

    private ArmMotorSubsystem armMotor;

    private BucketSubsystem bucket;

    private DipperSubsystem dipper;

    private BucketPivotSubsystem bucketPivot;

    private DroneSubsystem drone;

    GamepadEx gpad1;

    @Override
    public void initialize() {

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        gpad1 = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorSubsystem(hardwareMap);
        bucket = new BucketSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);


        // Run Intake and also Intake Pixels.
        gpad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
             .whenHeld(
                     new InstantCommand(() -> {
                         intake.runIntake();
                         bucket.intakePixels();
                     })
             )
             .whenReleased(new InstantCommand(() -> {
                 intake.stopIntake();
                 bucket.stopBucket();
             }));


        // Dispense Pixels.
        gpad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
             .whenHeld(new InstantCommand(() -> {
                 bucket.dispensePixels();
                 intake.dispenseIntake();

             }))
             .whenReleased(new InstantCommand(() -> {
                 bucket.stopBucket();
                 intake.stopIntake();
             }));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(2200);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
             }));


        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(1500);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
             }));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
             .whenPressed(new InstantCommand(() -> {
                 armMotor.setArmToPos(0);
                 dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                 int timeout = 1200;
                 int epsilon = 500; // Machine epsilon
                 while (!(-epsilon < armMotor.getAvgArmPosition() && armMotor.getAvgArmPosition() < epsilon)) {
                     try {
                         Thread.sleep(20);
                     } catch (InterruptedException e) {
                         throw new RuntimeException(e);
                     }
                 }
                 bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
             }));

//        gpad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new InstantCommand(() -> {
//                    drone.shootDrone();
//                }));
//
//        gpad1.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand(() -> {
//                    drone.loadDrone();
//                }));

        schedule(new RunCommand(() -> {

            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            double Speedper = 1;

            drive.drive(
                    forward * Speedper,
                    strafe * Speedper,
                    spin * Speedper
            );


        }).alongWith(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
            telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
            telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
            telemetry.addData("Correction", armMotor.correction);
//            telemetry.addData("DipperRightServo", dipper.rightDipperServo.getPosition());
//            telemetry.addData("DipperLeftServo", dipper.leftDipperServo.getPosition());
            telemetry.update();
        })));
    }

}
