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
import org.firstinspires.ftc.teamcode.subsystem.BucketSenseSubsystem;

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

        RevBlinkinLedDriver blinker = hardwareMap.get(RevBlinkinLedDriver.class, "blinker");

        BucketSenseSubsystem bucketSense = new BucketSenseSubsystem(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {


            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            if (gamepad1.dpad_up) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            if (gamepad1.dpad_down) {
                blinker.setPattern((RevBlinkinLedDriver.BlinkinPattern.YELLOW));
            }

            if (bucketSense.isYellow()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if(bucketSense.isWhite()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else if(bucketSense.isPurple()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if(bucketSense.isGreen()) {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
            }
            
            telemetry.addData("RGB", bucketSense.normalizedValues());
            telemetry.update();
        }
    }
}
