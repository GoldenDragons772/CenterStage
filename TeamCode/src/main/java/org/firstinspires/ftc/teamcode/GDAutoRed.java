package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "GDAutoRed")
public class GDAutoRed extends LinearOpMode {


    public void runOpMode() {
        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
        // Lists the trajectories

        // Long Distance Red
        Trajectory ld_red = mecDrive.trajectoryBuilder(new Pose2d(60, -34, Math.toRadians(0)))
                .strafeLeft(100)
                .build();

        // Short Distance Red
        Trajectory sd_red = mecDrive.trajectoryBuilder(new Pose2d(60, 34, Math.toRadians(0)))
                .strafeLeft(22)
                .build();
        waitForStart();

        mecDrive.followTrajectory(ld_red);
        tel.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        tel.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        tel.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());

    }
}
