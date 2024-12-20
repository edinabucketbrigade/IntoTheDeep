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
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or
 * Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at
 * <a href="https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html">...</a>
 * Note that a Mecanum drive must display an X roller-pattern when viewed from
 * above.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes
 * (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial: Driving forward and backwards Left-joystick Forward/Backwards
 * 2) Lateral: Strafing right and left Left-joystick Right and Left
 * 3) Yaw: Rotating Clockwise and counter clockwise Right-joystick Right and
 * Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed
 * for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left
 * stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * This opmode also uses ftclib to manage the gamepad. It handles s lot of things
 * that you would normally have to program yourself, like button bounce and
 * toggling. The installation instructions are here:
 * <a href="https://docs.ftclib.org/ftclib/v/v2.0.0/installation">...</a>
 * There are other libraries available that perform similar functions.
 */

@TeleOp(name = "Omni/Mecanum Test", group = "Test")
//@Disabled
public class EdinaFTCOmniMecanumTest extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    public RobotHardware robot = new RobotHardware(this);

    private final ElapsedTime runtime = new ElapsedTime();
    boolean testMode = false;
    boolean motorForward = true;
    double power;
    GamepadEx gamePadEx;

    @Override
    public void runOpMode() {
        robot.init();
        gamePadEx = new GamepadEx(gamepad1);
        ToggleButtonReader startToggle = new ToggleButtonReader(gamePadEx, GamepadKeys.Button.START);
        ToggleButtonReader directionToggle = new ToggleButtonReader(gamePadEx, GamepadKeys.Button.LEFT_BUMPER);

        robot.leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Use the Start button to toggle test mode.");
        telemetry.addData("", "Use left bumper to toggle direction.");
        telemetry.addData("", "Use X, A, Y or B to run each motor.");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            startToggle.readValue();
            directionToggle.readValue();
            // Start toggles test mode to use x, a, y and b to test each motor.
            testMode = startToggle.getState();
            // Switch motor direction.
            motorForward = directionToggle.getState();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's
            // power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
            double leftFrontPower = (axial + lateral + yaw) / denominator;
            double rightFrontPower = (axial - lateral - yaw) / denominator;
            double leftBackPower = (axial - lateral + yaw) / denominator;
            double rightBackPower = (axial + lateral - yaw) / denominator;

            // This is test code:
            //
            // Each button should make the corresponding motor run FORWARD.
            // 1) First get all the motors to their correct positions on the robot
            // by adjusting your Robot Configuration if necessary.
            // 2) Then make sure they run in the correct direction by modifying the
            // the setDirection() calls above.
            if (testMode) {
                power = motorForward ? 1.0 : -1.0;
                leftFrontPower = gamepad1.x ? power : 0.0; // X gamepad
                leftBackPower = gamepad1.a ? power : 0.0; // A gamepad
                rightFrontPower = gamepad1.y ? power : 0.0; // Y gamepad
                rightBackPower = gamepad1.b ? power : 0.0; // B gamepad
            }

            // Send calculated power to wheels
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Test Mode", testMode);
            telemetry.addData("Forward", motorForward);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Front Ticks Left/right", "%d, %d",
                    robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition());
            telemetry.addData("Back Ticks Left/right", "%d, %d",
                    robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
