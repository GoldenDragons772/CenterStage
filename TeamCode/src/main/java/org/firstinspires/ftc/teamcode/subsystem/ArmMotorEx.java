package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmMotorEx implements Subsystem {

    private final DcMotorEx leftArmMotor, rightArmMotor;

    public enum ArmPos {
        BACKBOARD_TOP,
        BACKBOARD_CENTER,
        BACKBOARD_LOW
    }

    public ArmMotorEx(HardwareMap hw) {
        // Get the Motor
        leftArmMotor = hw.get(DcMotorEx.class, "LeftArmMotor");
        rightArmMotor = hw.get(DcMotorEx.class, "RightArmMotor");


        // Break the Motors
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse Motors
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setArmToPos(int pos) {
        leftArmMotor.setTargetPosition(pos);
        rightArmMotor.setTargetPosition(pos);

        // Set Mode of the Motor
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
