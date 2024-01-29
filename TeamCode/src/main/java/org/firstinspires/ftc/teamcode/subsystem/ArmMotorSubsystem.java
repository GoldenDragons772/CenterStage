package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmMotorSubsystem implements Subsystem {
    public double correction;
    private ArmPos armPos = ArmPos.HOME;
    public DcMotorEx leftArmMotor, rightArmMotor;


    public enum ArmPos {
        HIGH(2200),
        MIDDLE(1500),
        LOW(750),
        HOME(0);
        final private int position;

        ArmPos(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public ArmMotorSubsystem(HardwareMap hw) {
        // Get the Motor
        leftArmMotor = hw.get(DcMotorEx.class, "LeftArmMotor");
        rightArmMotor = hw.get(DcMotorEx.class, "RightArmMotor");

        // Break the Motors
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse Motors
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && leftArmMotor.isBusy() && rightArmMotor.isBusy());
    }

    public void setArmToPos(ArmPos pos) {
        this.setArmToPos(pos.getPosition());
    }

    public void setArmToPos(int pos) {
        leftArmMotor.setTargetPosition(pos + 50);
        rightArmMotor.setTargetPosition(pos);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(0.8);
        rightArmMotor.setPower(0.8);
//        if(pos == 0) {
//            // A Timeout just in case something breaks.
//            long startTime = System.currentTimeMillis();
//            long timeout = 3500; // 3.5 seconds in milliseconds
//
//            while(getAvgArmPosition() > 0 && (System.currentTimeMillis() - startTime) < timeout) {
//                leftArmMotor.setPower(0.8);
//                rightArmMotor.setPower(0.8);
//            }
//            // Stop the Motor
//            leftArmMotor.setPower(0);
//            rightArmMotor.setPower(0);
//            // Reset Positions
//            rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        } else {
//            leftArmMotor.setPower(0.8);
//            rightArmMotor.setPower(0.8);
//        }
    }

    public void stopResetArm() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

  public int getAvgArmPosition() {
        return (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition()) / 2;
    }
    public ArmPos getArmPos(){
        return armPos;
    }
}
