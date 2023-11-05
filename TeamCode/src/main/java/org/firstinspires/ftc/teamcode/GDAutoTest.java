package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "GDAutoTest")
public class GDAutoTest extends LinearOpMode {


    public void runOpMode() {
        Telemetry tel = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
        // Lists the trajectoreis
        Trajectory t = mecDrive.trajectoryBuilder(
                        new Pose2d(-20, -20, Math.toRadians(-180)))
                .splineTo(new Vector2d(-20, 20), Math.toRadians(0))
                .splineTo(new Vector2d(20, 20), Math.toRadians(0))
                .splineTo(new Vector2d(20, -20), Math.toRadians(-180))
                .splineTo(new Vector2d(-20, -20), Math.toRadians(-180))
                .build();

        waitForStart();

        mecDrive.followTrajectory(t);
        tel.addData("x", mecDrive.getLocalizer().getPoseEstimate().getX());
        tel.addData("y", mecDrive.getLocalizer().getPoseEstimate().getY());
        tel.addData("heading", mecDrive.getLocalizer().getPoseEstimate().getHeading());

    }
}
