package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;


import java.io.FileNotFoundException;
import java.io.FileReader;

@TeleOp(name = "Duo", group = "TeleOp")
public class Duo extends CommandOpMode {

    private MecanumDriveSubsystem drive;

    private IntakeSubsystem intake;

    private ArmMotorEx armMotor;

    private BucketSubsystem bucket;

    private DipperSubsystem dipper;

    private BucketPivotSubsystem bucketPivot;

    private DroneSubsystem drone;

    GamepadEx gpad1;

    JsonObject bindings;
    @Override
    public void initialize() {
        JsonParser parser = new JsonParser();
        try {
            bindings = (JsonObject) parser.parse(new FileReader("res/values/bindings.json"));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        drive = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);
        gpad1 = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorEx(hardwareMap);
        bucket = new BucketSubsystem(hardwareMap);
        dipper = new DipperSubsystem(hardwareMap);
        bucketPivot = new BucketPivotSubsystem(hardwareMap);
        drone = new DroneSubsystem(hardwareMap);


        // Run Intake and also Intake Pixels.
        gpad1.getGamepadButton(getBinding("intakeCollect"))
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
        gpad1.getGamepadButton(getBinding("intakeDispense"))
                .whenHeld(new InstantCommand(() -> {
                    bucket.dispensePixels();
                    intake.dispenseIntake();

                }))
                .whenReleased(new InstantCommand(() -> {
                    bucket.stopBucket();
                    intake.stopIntake();
                }));

        // Extend arm up
        gpad1.getGamepadButton(getBinding("armExtend"))
                .whenPressed(new InstantCommand(() -> {
                    armMotor.setArmToPos(1500);
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.SCORING_POSITION);
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.DROPPING_POS);
                }));
        // Retract arm down
        gpad1.getGamepadButton(getBinding("armRetract"))
                .whenPressed(new InstantCommand(() -> {
                    armMotor.setArmToPos(0);
                    dipper.setDipperPosition(DipperSubsystem.DipperPositions.LOADING_POSITION);
                    bucketPivot.runBucketPos(BucketPivotSubsystem.BucketPivotPos.LOADING_POS);
                }));
//        // Shoot drone
//        gpad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new InstantCommand(() -> {
//                    drone.shootDrone();
//                }));
//        // Load drone
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

    GamepadKeys.Button getBinding(String action){
        String stringBinding = bindings.get(action).getAsString().toUpperCase();
        return GamepadKeys.Button.valueOf(stringBinding);
    }

}
