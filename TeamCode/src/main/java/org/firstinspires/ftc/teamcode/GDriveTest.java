package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@TeleOp(name = "GDriveTest", group = "Teleop")
public class GDriveTest extends LinearOpMode {
    TouchSensor touch;
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        touch = hardwareMap.get(TouchSensor.class, "touch");

        waitForStart();

        while(opModeIsActive()) {

            if (touch.isPressed()) {
                drive.setWeightedDrivePower(new Pose2d(
                        1,
                        1,
                        1
                ));
            } else {
                drive.setWeightedDrivePower(new Pose2d(
                        0,
                        0,
                        0
                ));
            }
        }
    }
}
