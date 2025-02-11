/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

/*
 * This is a teleop opMode designed to work with a RobotHardware class and the FtcLib library.
 * The main use of FtcLib is the gamepad extensions.
 */

@TeleOp(name = "Driver Control", group = "Robot")
//@Disabled
public class DriverControl extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    public RobotHardware robot = new RobotHardware(this);
    private final Lift lift = new Lift(robot);
    private final Slide slide = new Slide(robot);
    private final Bucket bucket = new Bucket(robot);
    private final Claw claw = new Claw(robot);
    private final Wrist wrist = new Wrist(robot);
    private final Rotate rotate = new Rotate(robot);

    // Allow selecting driver or field centric control.
    private boolean isFieldCentric = false;
    // Use the FtcLib gamepad extension.
    GamepadEx gamepadOne = null;
    GamepadEx gamepadTwo = null;
    TriggerReader triggerReaderLeft;
    TriggerReader triggerReaderRight;

    @Override
    public void runOpMode() {
        // Gamepad A
        gamepadOne = new GamepadEx(gamepad1);
        // Gamepad B
        gamepadTwo = new GamepadEx(gamepad2);

        triggerReaderLeft = new TriggerReader(gamepadTwo, GamepadKeys.Trigger.LEFT_TRIGGER);
        triggerReaderRight = new TriggerReader(gamepadTwo, GamepadKeys.Trigger.RIGHT_TRIGGER);
        robot.init();
        lift.init();
        slide.init();
        bucket.init();
        claw.init();
        wrist.init();
        rotate.init();
        wrist.setRotate(rotate);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamepadOne.readButtons();
            gamepadTwo.readButtons();
            triggerReaderLeft.readValue();
            triggerReaderRight.readValue();

            // gamepadOne (A on the driver hub)
            wrist.setProperties(gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                    gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT));

            // Rotate based on wrist position.
            setRotate();

            // Allow manual control of rotate.
            rotate.setProperties(gamepadOne.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                    gamepadOne.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER));

            // Both joysticks pressed switch driver/field centric control.
            if (gamepadOne.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON) && gamepadOne.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                isFieldCentric = !isFieldCentric;
            }

            // gamepadTwo (B on the driver hub)
            lift.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.A),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.X),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.Y));

            bucket.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP));

            claw.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER));

            // Left trigger reverses the slide motor. Using both triggers will add the results
            // together.
            slide.setProperties(-gamepadTwo.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) +
                    gamepadTwo.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            boolean x = slide.extended();
            double axial = RobotHardware.ScaleMotorSquare(-gamepadOne.getLeftY(), x);  // Note: pushing stick forward gives negative value
            double lateral = RobotHardware.ScaleMotorSquare(gamepadOne.getLeftX(), x);
            double yaw = RobotHardware.ScaleMotorSquare(gamepadOne.getRightX(), x);

            robot.moveRobot(axial, lateral, yaw);

            if (isFieldCentric) {
                robot.moveRobot(axial, lateral, yaw);
            } else {
                robot.moveRobot(axial, lateral, yaw, robot.imu.getRobotYawPitchRollAngles().getYaw());
            }

            lift.update();
            slide.update();
            bucket.update();
            claw.update();
            wrist.update();
            rotate.update();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Field Centric", isFieldCentric);
            telemetry.addData("Lift State", lift.liftState);
            telemetry.addData("Lift Target", "%d", robot.liftMotor.getTargetPosition());
            telemetry.addData("Lift Position", "%d", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Lift Power", "%6.2f", robot.liftMotor.getPower());
            telemetry.addData("Current", robot.liftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Lift Busy", robot.liftMotor.isBusy());
            telemetry.addData("Lift Mode", robot.liftMotor.getMode());
            telemetry.addData("Lift PIDF Run To Position", robot.liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.addData("Slide State", slide.slideState);
            telemetry.addData("Slide Target", "%d", robot.slideMotor.getTargetPosition());
            telemetry.addData("Slide position", "%d", robot.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Power", "%6.2f", robot.slideMotor.getPower());
            telemetry.addData("Slide Busy", robot.slideMotor.isBusy());
            telemetry.addData("Slide Mode", robot.slideMotor.getMode());
            telemetry.addData("Slide PIDF Run Using Encoder", robot.slideMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Claw position", "%1.2f", claw.claw.getPosition());
            telemetry.addData("Bucket position", "%1.2f", bucket.bucket.getPosition());
            telemetry.addData("Wrist position", "%1.2f", wrist.wrist.getPosition());
            telemetry.update();
        }
    }

    private void setRotate() {
        // If either is true driver wants to control rotate.
        if (!rotate.DPAD_LEFT && !rotate.DPAD_RIGHT) {
            switch (wrist.wristState) {
                case Down:
                case Neutral:
                    rotate.setProperties(true, false);
                    break;
                case Up:
                    rotate.setProperties(false, true);
                    break;
            }
        }
    }
}
