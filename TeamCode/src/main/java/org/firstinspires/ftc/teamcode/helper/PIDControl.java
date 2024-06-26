package org.firstinspires.ftc.teamcode.helper;

public class PIDControl {
    private double lastError = 0;
    private double lastTime = 0;

    private static double KP = 0.2;
    private static double KD = .28;
    private static double KI = 0.0;

    public double PID(double error) {
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