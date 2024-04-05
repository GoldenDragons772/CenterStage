package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BucketSenseSubsystem extends SubsystemBase {

    public RevColorSensorV3 colorSensorV3;
    // Base Color Threshold
    public final double RED_THRESHOLD_LEFT = 700, RED_THRESHOLD_RIGHT = 700;
    public final double BLUE_THRESHOLD_LEFT = 700, BLUE_THRESHOLD_RIGHT = 700;
    public final double RED_THRESHOLD = 550;
    public final double BLUE_THRESHOLD = 550;
    public final double GREEN_THRESHOLD = 550;
    // Complex Color Thresholds (not red, green, or blue)
    public final double YELLOW_RED_VALUE = 0.37;
    public final double YELLOW_GREEN_VALUE = 0.50;
    public final double YELLOW_BLUE_VALUE = 0.13;
    public final double[] YELLOW_CONSTANTS = {YELLOW_RED_VALUE, YELLOW_GREEN_VALUE, YELLOW_BLUE_VALUE};
    public final double YELLOW_THRESHOLD = 0.03;

    public final double PURPLE_RED_VALUE = 0.27;
    public final double PURPLE_GREEN_VALUE = 0.24;
    public final double PURPLE_BLUE_VALUE = 0.47;

    public final double[] PURPLE_CONSTANTS = {PURPLE_RED_VALUE, PURPLE_GREEN_VALUE, PURPLE_BLUE_VALUE};

    public final double PURPLE_THRESHOLD = 0.3;



    public final double WHITE_RED_VALUE = 0.30125;
    public final double WHITE_GREEN_VALUE = 0.37125;
    public final double WHITE_BLUE_VALUE = 0.325;
    public final double[] WHITE_CONSTANTS = {WHITE_RED_VALUE, WHITE_GREEN_VALUE, WHITE_BLUE_VALUE};

    public final double GREEN_RED_VALUE = 0.18;
    public final double GREEN_GREEN_VALUE = 0.50;
    public final double GREEN_BLUE_VALUE = 0.33;
    public final double[] GREEN_CONSTANTS = {GREEN_RED_VALUE, GREEN_GREEN_VALUE, GREEN_BLUE_VALUE};
    public final double WHITE_THRESHOLD = 0.12;
    public final double WHITE_TOTAL_COUNT = 800;

    // check for black with the alpha value
    public final double BLACK_ALPHA_VALUE = 325; //Test value

    public BucketSenseSubsystem(HardwareMap hw){
        this.colorSensorV3 = hw.get(RevColorSensorV3.class, "bucketsense");
    }

    public double distance(){
        return colorSensorV3.getDistance(DistanceUnit.CM);
    }


    // utility method for easily getting the color value.
    public int red(){ return colorSensorV3.red(); }
    public int green(){ return colorSensorV3.green(); }
    public int blue(){ return colorSensorV3.blue(); }

    public double total(){ return red() + green() + blue(); }

    // convert to array
    public double[] rgb(){
        double[] arr = new double[3];
        arr[0] = red();
        arr[1] = green();
        arr[2] = blue();

        return arr;
    }

    // Scale the rgb values (0 to 1)
    public double[] normalizedRGB(){
        double[] arr = new double[3];
        double[] originalArr = rgb();

        double total = 0;
        for(double i : originalArr)
            total+= i;

        for(int i = 0; i < 3; i++){
            arr[i] = originalArr[i] / total;
        }

        return arr;
    }

    public double arrayError(double[] arr1, double[] arr2){
        double total = 0;

        for(int i = 0; i < arr1.length; i++){
            total += Math.pow(arr1[i] - arr2[i], 2);
        }

        return Math.sqrt(total);
    }

    // Color Error Methods
    public double yellowError(){ return arrayError(normalizedRGB(), YELLOW_CONSTANTS); }
    public double whiteError(){ return arrayError(normalizedRGB(), WHITE_CONSTANTS); }

    public double purpleError(){ return arrayError(normalizedRGB(), PURPLE_CONSTANTS); }

    public boolean isBlack(){
        return (colorSensorV3.alpha() < BLACK_ALPHA_VALUE) ? true : false;
    }

    public int alphaValue(){
        return colorSensorV3.alpha();
    }

    public boolean isRed(){
        return colorSensorV3.red() > RED_THRESHOLD;
    }

    public boolean isBlue(){
        return colorSensorV3.blue() > BLUE_THRESHOLD;
    }

    public boolean isGreen(){
        return colorSensorV3.green() > GREEN_THRESHOLD;
    }

    public boolean isYellow(){
        return yellowError() < YELLOW_THRESHOLD && (whiteError() > 0.02);
    }

    public boolean isPurple() {
        return purpleError() < PURPLE_THRESHOLD && (whiteError() > 0.02);
    }

    public boolean isWhite(){
        return whiteError() < WHITE_THRESHOLD && total() > WHITE_TOTAL_COUNT;
    }

    public String normalizedValues() {
        double red = colorSensorV3.red();
        double green = colorSensorV3.green();
        double blue = colorSensorV3.blue();

        double total = red + green + blue;
        return String.format("RGB: %.2f %.2f %.2f", red / total, green / total, blue / total);
    }

    // turn on the lights
    public void enableLED(boolean LEDMode){
        colorSensorV3.enableLed(LEDMode);
    }

    public boolean withinColorRange(){
        return isYellow() || isWhite();
    }
}
