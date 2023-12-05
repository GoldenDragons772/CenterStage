//package org.firstinspires.ftc.teamcode.Subsystem;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class ArmDriver {
//
//    Thread armPosThread;
//
//    private DcMotorEx leftArmMotor, rightArmMotor;
//
//    public enum ArmPos {
//        HIGH,
//        CENTER,
//        LOW,
//    }
//
//    ArmPos currentPos = ArmPos.LOW;
//    private HardwareMap hardwareMap;
//    public ArmDriver(HardwareMap hm, ArmPos pos) {
//        hardwareMap = hm;
//        currentPos = pos;
//
//        // Get the Motors
//        leftArmMotor = hardwareMap.get(DcMotorEx.class, "LeftArmMotor");
//        rightArmMotor = hardwareMap.get(DcMotorEx.class, "RightArmMotor");
//
//        // Brake the Motor when Power is 0
//        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set the Mode of the Motor
//        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    class ArmPosThread implements Runnable {
//        @Override
//        public void run() {
//            switch (currentPos) {
//                case HIGH:
//                    leftArmMotor.setTargetPosition(1);
//                    rightArmMotor.setTargetPosition(1);
//                    break;
//                case CENTER:
//                    leftArmMotor.setTargetPosition(2);
//                    rightArmMotor.setTargetPosition(2);
//                    break;
//                case LOW:
//                    leftArmMotor.setTargetPosition(3);
//                    rightArmMotor.setTargetPosition(3);
//                    break;
//            }
//        }
//    }
//
//    public void setArmPos(ArmPos pos) {
//        currentPos = pos;
//
//        if(armPosThread.isAlive()) {
//            armPosThread.interrupt();
//        } else {
//            armPosThread = new Thread(new ArmPosThread());
//            armPosThread.start();
//        }
//    }
//}
