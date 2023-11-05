package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;
@Autonomous(name = "GDAuto")
public class GDAuto extends LinearOpMode {

    GDAutoPresets.AUTO auto;
    int currentAuto = 0;
    String autoName = "NONE";

    Trajectory currentTraj;


    @Override
    public void runOpMode() {
        //Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
        // Lists the trajectories

        // Long Distance Blue
        Trajectory ld_blue = mecDrive.trajectoryBuilder(new Pose2d(-60, -34, Math.toRadians(0)))
                .strafeLeft(100)
                .build();

        // Short Distance Blue
        Trajectory sd_blue = mecDrive.trajectoryBuilder(new Pose2d(-60, 34, Math.toRadians(0)))
                .strafeLeft(22)
                .build();

        // Long Distance Red
        Trajectory ld_red = mecDrive.trajectoryBuilder(new Pose2d(60, -34, Math.toRadians(0)))
                .strafeLeft(100)
                .build();

        // Short Distance Red
        Trajectory sd_red = mecDrive.trajectoryBuilder(new Pose2d(60, 34, Math.toRadians(0)))
                .strafeLeft(22)
                .build();



        while(opModeInInit()) {
            // Auto Selector
            if(gamepad1.dpad_right) { // Long Distance Red Auto
                currentAuto = GDAutoPresets.AUTO.LD_RED;
                currentTraj = ld_red;
                autoName = "LD_RED";
            } else if(gamepad1.dpad_left) { // Short Distance Red Auto
                currentAuto = GDAutoPresets.AUTO.SD_RED;
                currentTraj = sd_red;
                autoName = "SD_RED";
            } else if(gamepad1.dpad_up) { // Long Distance Blue Auto
                currentAuto = GDAutoPresets.AUTO.LD_BLUE;
                currentTraj = ld_blue;
                autoName = "LD_BLUE";
            } else if(gamepad1.dpad_down) { // Short Distance Blue Auto
                currentAuto = GDAutoPresets.AUTO.SD_BLUE;
                autoName = "SD_BLUE";
                currentTraj = sd_blue;
            }
            telemetry.addData("SELECTED AUTO", autoName);
            telemetry.update();
        }

        waitForStart();

        mecDrive.followTrajectory(currentTraj);
        telemetry.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        telemetry.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        telemetry.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());
    }
}
