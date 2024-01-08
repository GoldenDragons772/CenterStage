package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;

@Autonomous(name = "GDAuto")
public class GDAuto extends LinearOpMode {

    GDAutoPresets.AUTO auto;
    int currentAuto = 0;
    String autoName = "NONE";

    Pose2d currentStartPos = new Pose2d(0, 0, Math.toRadians(0));

    Trajectory currentTraj;
    Pose2d LD_RED_STARTPOS = new Pose2d(-37, -62, Math.toRadians(0));
    Pose2d LD_BLUE_STARTPOS = new Pose2d(-37, 62, Math.toRadians(0));

    Pose2d SD_RED_STARTPOS = new Pose2d(15, -60, Math.toRadians(0));

    Pose2d SD_BLUE_STARTPOS = new Pose2d(0, 0, Math.toRadians(0));


    StorePos pos = new StorePos();


    @Override
    public void runOpMode() {
        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MainMecanumDrive mecDrive = new MainMecanumDrive(hardwareMap);

        // Long Distance Blue
        Trajectory ld_blue = mecDrive.trajectoryBuilder(new Pose2d(-37, 62, Math.toRadians(0)))
                .strafeRight(30)
                .splineToConstantHeading(new Vector2d(54, 18), Math.toRadians(10))
                .build();

        // Short Distance Blue
        Trajectory sd_blue = mecDrive.trajectoryBuilder(new Pose2d(15, 62, Math.toRadians(0)))
                //.strafeLeft(22)
                .forward(32)
                .build();

        // Long Distance Red
        Trajectory ld_red = mecDrive.trajectoryBuilder(new Pose2d(-37, -62, Math.toRadians(0)))
                .strafeLeft(47)
                .splineToConstantHeading(new Vector2d(47, -26), Math.toRadians(170))
                .build();

        // Short Distance Red
        Trajectory sd_red = mecDrive.trajectoryBuilder(new Pose2d(15, -60, Math.toRadians(0)))
                .forward(35)
                .build();

        while(opModeInInit()) {
            // Auto Selector
            if(gamepad1.dpad_right) { // Long Distance Red Auto
                currentAuto = GDAutoPresets.AUTO.LD_RED;
                currentTraj = ld_red;
                currentStartPos = LD_RED_STARTPOS;
                autoName = "LD_RED";
            } else if(gamepad1.dpad_left) { // Short Distance Red Auto
                currentAuto = GDAutoPresets.AUTO.SD_RED;
                currentTraj = sd_red;
                currentStartPos = SD_RED_STARTPOS;
                autoName = "SD_RED";
            } else if(gamepad1.dpad_up) { // Long Distance Blue Auto
                currentAuto = GDAutoPresets.AUTO.LD_BLUE;
                currentTraj = ld_blue;
                currentStartPos = LD_BLUE_STARTPOS;
                autoName = "LD_BLUE";
            } else if(gamepad1.dpad_down) { // Short Distance Blue Auto
                currentAuto = GDAutoPresets.AUTO.SD_BLUE;
                autoName = "SD_BLUE";
                currentStartPos = SD_BLUE_STARTPOS;
                currentTraj = sd_blue;
            }
            telemetry.addData("SELECTED AUTO", autoName);
            telemetry.update();
        }

        waitForStart();
        // Set the Start Pose of the Robot
        mecDrive.setPoseEstimate(currentStartPos);
        mecDrive.followTrajectory(currentTraj);
        pos.StorePos(mecDrive.getPoseEstimate());
        telemetry.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        telemetry.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        telemetry.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());
    }
}
