package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    OpenCvCamera webcam;

    TagPipeline tag = new TagPipeline();

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "GoldenCamera"),
                cameraMonitorViewId
        );

        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        webcam.setPipeline(tag);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while(opModeIsActive()) {

            int correction = tag.correction();

            if(correction > 20) {
                drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0,
                            0.45
                    )
                );
            } else if (correction < -20) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                -0.45
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
            }

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry telemetry = dashboard.getTelemetry();

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Correction", tag.correction());

            TelemetryPacket packet = new TelemetryPacket();
            telemetry.update();
        }
    }
}


