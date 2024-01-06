package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

// Sample code
// https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples

@Disabled
@Config
@Autonomous(name = "RobotTag", group = "Auto")
public class RoboTag extends LinearOpMode {
    int centerX = 160;
    private HuskyLens huskyLens;
    private double lastError = 0;
    private double lastTime = 0;

    public static double KP = 0.2;
    public static double KD = .28;
    public static double KI = 0.0;

    /**
     * Returns the error in pixels away from the center of the screen
     */
    private int error(int blockXValue) {
        return blockXValue - centerX;
    }

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "hus   ky");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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
            double time = System.currentTimeMillis();
            if (blocks.length == 0) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                continue;
            }
            HuskyLens.Block block = blocks[0];
            int error = error(block.x);

            double pid = -(PID(error) / 45);
            drive.setWeightedDrivePower(new Pose2d(0, 0, pid));
            telemetry.addData("BlockData", block.toString());
            telemetry.addData("Error", error);
            telemetry.addData("PID", String.valueOf(pid*50));
            telemetry.update();
        }
    }

    /**
     * Returns PID based on error and time deltas.
     */
    private double PID(double error) {
        double i = 0;
        double time = System.currentTimeMillis();

        final double maxI = 1;

        double p = KP * error;
        i += KI * (error * (time - lastTime));

        if (i > maxI) {
            i = maxI;
        }
        if (i < -maxI) {
            i = -maxI;
        }

        double d = KD * (error - lastError) / (time - lastTime);
        lastError = error;
        lastTime = time;
        return d + i + p;
    }
}


