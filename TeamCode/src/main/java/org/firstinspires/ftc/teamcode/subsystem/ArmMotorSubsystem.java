package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

@Config
public class ArmMotorSubsystem implements Subsystem {
    public double correction;
    private ArmPos armPos = ArmPos.HOME;
    public DcMotorEx leftArmMotor, rightArmMotor;
    Timer t;
    TimerTask tt;

    public enum ArmPos {
        HIGH(2200),
        MIDDLE(1250),
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

    public void stopResetArm() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArmToPos(int pos) {
        leftArmMotor.setTargetPosition(pos);
        rightArmMotor.setTargetPosition(pos);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(0.8);
        rightArmMotor.setPower(0.8);

        if(pos == 0) {

            t = new Timer();
            tt = new TimerTask() {
                @Override
                public void run() {
                    stopResetArm();
                };
            };

            t.schedule(tt,3500);
        } else {
            try {
                tt.cancel();
            } catch (Exception e) {
                // Do Nothing
            }
        }
    }



  public int getAvgArmPosition() {
        return (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition()) / 2;
    }
    public ArmPos getArmPos(){
        return armPos;
    }
}
