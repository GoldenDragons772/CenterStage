package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@Autonomous(name = "GDAutoBlue")
public class GDAutoBlue extends LinearOpMode {


    public void runOpMode() {
        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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
        waitForStart();

        mecDrive.followTrajectory(ld_blue);
        tel.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        tel.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        tel.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());

    }
}
