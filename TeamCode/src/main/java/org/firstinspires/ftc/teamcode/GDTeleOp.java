package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.Oak.CameraPreview;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@TeleOp(name = "GDTeleOp", group = "TeleOp")
public class GDTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        CRServo Servo1 = hardwareMap.crservo.get("Plane");
        //HuskyLens

        CameraPreview cam = new CameraPreview();
        String OakTestString = cam.OakPrint();


        waitForStart();
        while(opModeIsActive()) {

            // Square the Values to get better Control.
            double turn = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double speed = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -speed,
                            -turn,
                            -spin
                    )
            );

            if(gamepad1.triangle) {
                Servo1.setPower(0.8); // 7
            } else if(gamepad1.square) {
                Servo1.setPower(0.2);
            }

            drive.update();

            Pose2d currentPos = drive.getPoseEstimate();
            telemetry.addData("x", currentPos.getX());
            telemetry.addData("y", currentPos.getY());
            telemetry.addData("OakTest", OakTestString);

            telemetry.addData("turn", turn);
            telemetry.addData("speed", speed);
            telemetry.addData("spin", spin);
            telemetry.update();
        }
    }
}