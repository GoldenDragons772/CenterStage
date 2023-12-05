package org.firstinspires.ftc.teamcode.Oak;

public class CameraPreview {


    static {
        System.loadLibrary("GDOats");
    }

    public static native String OakPrint();

    public static native String startDevice();
}