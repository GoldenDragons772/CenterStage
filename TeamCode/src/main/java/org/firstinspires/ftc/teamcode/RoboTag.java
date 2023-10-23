package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RobotTag", group = "Auto")
public class RoboTag extends LinearOpMode {
    int centerX = 160;
    private HuskyLens huskyLens;
    private double lastError = 0;
    private double lastTime = 0;

    private int error(int blockXValue) {
        return blockXValue - centerX;
    }

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
            HuskyLens.Block block = blocks[0];
            int error = error(block.x);
            double pid = PID(error);
            telemetry.addData("BlockData", block.toString());
            telemetry.addData("PID", String.valueOf(pid));
            telemetry.update();

        }
    }

    private double PID(double error) {
        double i = 0;
        double time = System.currentTimeMillis();
        final double kP = 0.5;
        final double kI = 0.5;
        final double kD = 0.5;
        final double maxI = 1;

        double p = kP * error;

        i += kI * (error * (time - lastTime));

        if (i > maxI) {
            i = maxI;
        }
        if (i < -maxI) {
            i = -maxI;
        }

        double d = kD * (error - lastError) / (time - lastTime);
        lastError = error;
        lastTime = time;

        return d + i + p;
    }
}


