package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.*;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final CRServo bucketServo;

    public IntakeSubsystem(HardwareMap hw) {
        this.intakeMotor = hw.get(DcMotorEx.class, "IntakeMotor");
        this.bucketServo = hw.get(CRServo.class, "Bucket");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake() {
        intakeMotor.setPower(1);
        bucketServo.setPower(1);
    }

    public void spikePixel() {
        intakeMotor.setPower(0.2);
    }

    public void dispenseIntake() {
        intakeMotor.setPower(-0.2);
        bucketServo.setPower(-1);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        bucketServo.setPower(0);
    }
    public void specialDispenseJustForAutoPixelDispenseThing(){
        bucketServo.setPower(-1);
    }

}
