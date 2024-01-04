package org.firstinspires.ftc.teamcode.utils;

/** System for correcting error smoothly. Error is typically given as the difference between the current position and the desired position.
 * @see <a href="https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller">PID on wikipedia.</a>
 * */
public class PIDControl {
    private double lastError = 0;
    private double lastTime = 0;

    private static double KP = 0.2;
    private static double KD = .28;
    private static double KI = 0.0;

    /** For some odd reason we keep needing to use -(PID(error)/45) after this. Maybe we should have just used PID(error) and configured the values for that?.
     * */
    public double PID(double error) {
        double integral = 0;
        double time = System.currentTimeMillis();

        final double maxIntegral = 1;

        double proportional = KP * error;
        integral += KI * (error * (time - lastTime));

        // Clamp integral
        if (integral > maxIntegral) {
            integral = maxIntegral;
        }
        if (integral < -maxIntegral) {
            integral = -maxIntegral;
        }

        double derivative = KD * (error - lastError) / (time - lastTime);

        // Set the current values to be used in the next iteration.
        lastError = error;
        lastTime = time;

        return derivative + integral + proportional;
    }
}
