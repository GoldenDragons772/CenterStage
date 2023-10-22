package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.TagPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RobotTag", group = "Auto")
public class RoboTag extends LinearOpMode {
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {


        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        telemetry.update();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();
        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (HuskyLens.Block i : blocks) {
                telemetry.addData("Block", i.toString());
            }

            telemetry.update();

        }
    }
}


