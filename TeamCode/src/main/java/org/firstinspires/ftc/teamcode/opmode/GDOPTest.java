package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.drive.MainMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.subcommand.ArmPosCommand;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDriveSubsystem;

@TeleOp(name = "GDTeleOpTest", group = "TeleOp")
public class GDOPTest extends CommandOpMode {

    private MecanumDriveSubsystem drivetrain;
    private ArmMotorSubsystem armDriver;

    private GamepadEx gamepadEx1, gamepadEx2;


    @Override
    public void initialize() {
        // Initialize the Drive Train
        drivetrain = new MecanumDriveSubsystem(new MainMecanumDrive(hardwareMap), false);

        // Initialize ArmDriver
        armDriver = new ArmMotorSubsystem(hardwareMap);

        // Initialize the Gamepads
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Register Subsystem
        register(drivetrain, armDriver);

        // Set the Arm to the TOP position when DPad UP button is pressed
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ArmPosCommand(armDriver, ArmMotorSubsystem.ArmPos.BACKBOARD_TOP)
                );

        // Set the Arm to Center position when DPad LEFT button is pressed
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(
                        new ArmPosCommand(armDriver, ArmMotorSubsystem.ArmPos.BACKBOARD_CENTER)
                );

        // Set the Arm to LOW position when DPad DOWN button is pressed
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        new ArmPosCommand(armDriver, ArmMotorSubsystem.ArmPos.BACKBOARD_LOW)
                );
    }

    @Override
    public void run() {

        // Initialize the DriveSystem
        drivetrain.drive(
                gamepad1.right_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_stick_x * 0.30
        );
    }
}
