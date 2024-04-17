package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlinkinSubsystem extends SubsystemBase {

    public enum PixelColor {
        PIXEL_WHITE,
        PIXEL_GREEN,
        PIXEL_YELLOW,
        PIXEL_PURPLE
    }

    RevBlinkinLedDriver blinkin;


    public BlinkinSubsystem(HardwareMap hw) {
        blinkin = hw.get(RevBlinkinLedDriver.class, "blinker");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }

    public void setColor(PixelColor px) {
        switch(px) {
            case PIXEL_GREEN:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case PIXEL_WHITE:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
            case PIXEL_PURPLE:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
            case PIXEL_YELLOW:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
    }
}
