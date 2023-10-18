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

@Autonomous(name = "GDAutoTest")
public class GDAutoTest extends LinearOpMode {


    public void runOpMode() {
        Telemetry telemtry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = mecDrive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-20, -20), Math.toRadians(0))
                .splineTo(new Vector2d(20, 20), Math.toRadians(0))
                .splineTo(new Vector2d(0, 0), Math.toRadians(-50))
                .build();

        waitForStart();

        mecDrive.followTrajectory(traj);
        telemtry.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        telemtry.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        telemtry.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());

    }
}
