package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDControl;

@Config
public class ArmMotorSubsystem implements Subsystem {

    public static double kP, kD, kI;

    public double correction;

    public DcMotorEx leftArmMotor, rightArmMotor;
    PIDControl rightPID = new PIDControl();
    PIDControl leftPID = new PIDControl();

    public enum ArmPos {
        BACKBOARD_TOP,
        BACKBOARD_CENTER,
        BACKBOARD_LOW
    }

    public ArmMotorSubsystem(HardwareMap hw) {
        // Get the Motor
        leftArmMotor = hw.get(DcMotorEx.class, "LeftArmMotor");
        rightArmMotor = hw.get(DcMotorEx.class, "RightArmMotor");

        // Break the Motors
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse Motors
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set the PID Values
//        leftPID.KP = kP;
//        leftPID.KI = kI;
//        leftPID.KD = kD;

    }


    public void setArmToPos(int pos) {
        leftArmMotor.setTargetPosition(pos);
        rightArmMotor.setTargetPosition(pos);


        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        leftArmMotor.setVelocity(1000);
//        rightArmMotor.setVelocity(1000);
        leftArmMotor.setPower(0.8);
        rightArmMotor.setPower(0.8);
        
//        double minError = 0.5;
//        while (true) {
//            int rightArmError = rightArmMotor.getCurrentPosition() - pos;
//            int leftArmError = leftArmMotor.getCurrentPosition() - pos;
//            double rightArmCorrection = rightPID.PID(rightArmError);
//            correction = rightArmCorrection;
//            double leftArmCorrection = leftPID.PID(leftArmError);
//            if (((double) rightArmError + leftArmError) / 2 < minError) break;
//
//            // Run Motor
//            leftArmMotor.setPower(-leftArmCorrection / 50);
//            rightArmMotor.setPower(-rightArmCorrection / 50);
//        }
    }
    public int getAvgArmPosition(){
        return (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;
    }


}
