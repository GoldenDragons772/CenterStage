package org.firstinspires.ftc.teamcode.rr.drive.opmode;

import android.graphics.Color;

import androidx.annotation.Px;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@Disabled
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //MainMecanumDrive drive = new MainMecanumDrive(hardwareMap);

        RevBlinkinLedDriver blinker = hardwareMap.get(RevBlinkinLedDriver.class, "blinker");

        RevColorSensorV3 pxSense = hardwareMap.get(RevColorSensorV3.class, "bucketsense");
//
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {


            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            if(gamepad1.dpad_up) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            if(gamepad1.dpad_down) {
                blinker.setPattern((RevBlinkinLedDriver.BlinkinPattern.YELLOW));
            }

            NormalizedRGBA rgba = pxSense.getNormalizedColors();

            // Filter through colors

            // Check if Red
            if(pxSense.argb() > pxSense.blue() && pxSense.red() > pxSense.green()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            // Check if Green
            else if(pxSense.green() > pxSense.red() && pxSense.green() > pxSense.blue()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            // Check if Blue
            else if(pxSense.blue() > pxSense.red() && pxSense.blue() > pxSense.green()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
            }

            telemetry.addData("Red",  rgba.red);
            telemetry.addData("Blue", rgba.blue);
            telemetry.addData("Green", rgba.green);
            telemetry.update();
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -forward,
//                            -strafe,
//                            -spin
//                    )
//            );
//
//            drive.update();
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.update();
        }
    }
}
