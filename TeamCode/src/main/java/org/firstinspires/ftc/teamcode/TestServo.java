package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/*
    An iterative opMode used to test servos.
    The ServoImplEx class is used to try to extend the range of some servos.
    The behavior you see depends on the specific servo you are using. Read the specs for
    your servo and modify the test code to fit.
 */
@TeleOp(name = "Test Servo", group = "Test")
//@Disabled
public class TestServo extends OpMode {
    static final double INCREMENT = 0.1;     // amount to slew servo each button press.
    private Servo servo;
    private double position = 0;
    private GamepadEx gamepad;

    @Override
    public void init() {
        // Make the name match your config file and robot.
        servo = hardwareMap.get(ServoImplEx.class, "servo1");
        ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        gamepad = new GamepadEx(gamepad1);
        showTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        servo.setPosition(position);
    }

    @Override
    public void loop() {
        gamepad.readButtons();
        if (gamepad.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            position = 0;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            position = 1;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.Y)) {
            position = .5;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP) && position < 1) {
            position += INCREMENT;
        }

        if (gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN) && position > 0) {
            position -= INCREMENT;
        }

        servo.setPosition(position);
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("Left bumper = 0");
        telemetry.addLine("Right bumper = 1");
        telemetry.addLine("Y = .5 (middle)");
        telemetry.addLine("Dpad up: Increase position");
        telemetry.addLine("Dpad down: Decrease position");
        telemetry.addData("Position (not from servo)", position);
        telemetry.update();
    }
}
