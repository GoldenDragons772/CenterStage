package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@Autonomous(name = "GDAutoTest")
public class GDAutoTest extends LinearOpMode {


    public void runOpMode() {
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();



        mecanumDrive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-20, -20), Math.toRadians(0))
                .splineTo(new Vector2d(20, 20), Math.toRadians(0))
                .splineTo(new Vector2d(0, 0), Math.toRadians(-50))
                .build();
    }
}
