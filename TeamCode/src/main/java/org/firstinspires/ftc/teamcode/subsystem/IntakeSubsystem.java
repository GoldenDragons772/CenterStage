package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hw) {
        this.intakeMotor = hw.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake() {
        intakeMotor.setPower(1);
    }

    public void dispenseIntake() {
        intakeMotor.setPower(-0.5);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
