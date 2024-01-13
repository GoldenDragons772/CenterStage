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

@Deprecated
@Autonomous(name = "GDAuto")
public class GDAuto extends LinearOpMode {

    //    AutoPresets.AUTO auto;
    AutoPresets currentAuto;
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

        while (opModeInInit()) {
            // Auto Selector
            if (gamepad1.dpad_right) { // Long Distance Red Auto
                currentAuto = AutoPresets.LD_RED;
                currentStartPos = LD_RED_STARTPOS;
                autoName = "LD_RED";
            } else if (gamepad1.dpad_left) { // Short Distance Red Auto
                currentAuto = AutoPresets.SD_RED;
                currentStartPos = SD_RED_STARTPOS;
                autoName = "SD_RED";
            } else if (gamepad1.dpad_up) { // Long Distance Blue Auto
                currentAuto = AutoPresets.LD_BLUE;
                currentStartPos = LD_BLUE_STARTPOS;
                autoName = "LD_BLUE";
            } else if (gamepad1.dpad_down) { // Short Distance Blue Auto
                currentAuto = AutoPresets.SD_BLUE;
                autoName = "SD_BLUE";
                currentStartPos = SD_BLUE_STARTPOS;
            }
            currentTraj = getTrajectory(currentAuto, mecDrive);
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

    private Trajectory getTrajectory(AutoPresets preset, MainMecanumDrive mecDrive) {
        int startingRotation = 0; // in degrees
        int startingY = 62; // This is flipped across the x-axis for red.
        int farX = -37;
        int shortX = 15;
        boolean isAllianceBlue = preset.getAlliance() == AutoPresets.Alliance.BLUE;
        int sign = isAllianceBlue ? 1 : -1; // Sign to use for Y values. If the alliance is red, the trajectory is reflected across the x-axis.
        if (preset.getDistance() == AutoPresets.Distance.LONG) {
            return mecDrive.trajectoryBuilder(new Pose2d(farX, startingY * sign))
                           .strafeRight(30 * sign)
                           .splineToConstantHeading(new Vector2d(-23, 10 * sign), Math.toRadians(startingRotation))
                           .splineToConstantHeading(new Vector2d(60, 10 * sign), Math.toRadians(startingRotation))
                           .build();
        } else { // If distance is short.
            return mecDrive.trajectoryBuilder(new Pose2d(shortX, startingY * sign, Math.toRadians(startingRotation)))
                           .forward(35)
                           .build();
        }
    }
}
