package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneSubsystem extends SubsystemBase {

    public static double shootingPos = 0.75;

    public static double loadingPos = 0.2;

    Servo drone;

    public DroneSubsystem(HardwareMap hw) {
        drone = hw.get(Servo.class, "Plane");
    }

    public void shootDrone() {
        drone.setPosition(shootingPos);
    }

    public void loadDrone() {
        drone.setPosition(loadingPos);
    }
}
