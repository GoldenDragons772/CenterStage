package org.firstinspires.ftc.teamcode.opmode;

import android.content.res.Resources;
import android.graphics.RenderNode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;
import org.opencv.core.Mat;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Duo", group = "TeleOp")
public class Duo extends CommandOpMode {

    private MecanumDriveSubsystem drive;

    private IntakeSubsystem intake;

    private ArmMotorEx armMotor;
    GamepadEx gpad1;

    @Override
    public void initialize() {

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);
        gpad1 = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(hardwareMap);
        armMotor = new ArmMotorEx(hardwareMap);

        gpad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new InstantCommand(intake::runIntake))
                .whenReleased(new InstantCommand(intake::stopIntake));

        gpad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    armMotor.setArmToPos(1500);
                    int i = 0;

                    while(true) {
                        i++;
                        armMotor.setArmToPos((int)((Math.sin(i / 4)+0.5) * 250));

                        try {
                            TimeUnit.MILLISECONDS.sleep(20);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }));

        schedule(new RunCommand(() -> {

            double strafe = Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x);
            double forward = Math.pow(gamepad1.right_stick_y, 2) * Math.signum(gamepad1.right_stick_y);
            double spin = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x);

            double Speedper = 1;

            drive.drive(
                    forward * Speedper,
                    strafe * Speedper,
                    spin * Speedper
            );
        }).alongWith(new RunCommand(() -> {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("LeftArmPos", armMotor.leftArmMotor.getCurrentPosition());
            telemetry.addData("RightArmPos", armMotor.rightArmMotor.getCurrentPosition());
            telemetry.addData("PIDError", 1500 - (armMotor.leftArmMotor.getCurrentPosition() + armMotor.rightArmMotor.getCurrentPosition()) / 2);
            telemetry.addData("Correction", armMotor.correction);
            telemetry.update();
        })));
    }

}
