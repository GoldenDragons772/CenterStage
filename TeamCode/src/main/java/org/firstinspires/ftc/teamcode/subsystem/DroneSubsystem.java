package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubsystem extends SubsystemBase {

    Servo drone;

    public DroneSubsystem(HardwareMap hw) {
        drone = hw.get(Servo.class, "Plane");
    }

    public void shootDrone() {
        drone.setPosition(-1);
    }

    public void loadDrone() {
        drone.setPosition(1);
    }
}
