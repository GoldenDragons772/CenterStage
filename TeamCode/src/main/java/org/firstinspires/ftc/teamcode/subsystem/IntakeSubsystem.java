package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem.ArmPos.HOME;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final CRServo bucketServo;
    private final RevColorSensorV3 bucketSensor;

    public IntakeSubsystem(HardwareMap hw) {
        this.intakeMotor = hw.get(DcMotorEx.class, "IntakeMotor");
        this.bucketServo = hw.get(CRServo.class, "Bucket");
        this.bucketSensor = hw.get(RevColorSensorV3.class, "bucketSense");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake() {
        intakeMotor.setPower(0.9);
        bucketServo.setPower(0.9);
    }

    public void spikePixel() {
        intakeMotor.setPower(-0.2);
    }

    public void dispenseIntake() {
        if(ArmMotorSubsystem.armPos == HOME) {
            intakeMotor.setPower(-0.2);
        }
        bucketServo.setPower(-1);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        bucketServo.setPower(0);
    }
    public void specialDispenseJustForAutoPixelDispenseThing(){
        bucketServo.setPower(-1);
    }

    public BlinkinSubsystem.PixelColor getCurrentPixel() {

        BlinkinSubsystem.PixelColor detectedColor = BlinkinSubsystem.PixelColor.PIXEL_WHITE;
        NormalizedRGBA colorVal =  bucketSensor.getNormalizedColors();

        // TODO: Finish Writing Bucket sensor Code.

        return detectedColor;
    }
}
