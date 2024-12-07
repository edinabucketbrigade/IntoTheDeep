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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

/*
 * This is a teleop opMode designed to work with a RobotHardware class and the FtcLib library.
 * The main use of FtcLib is the gamepad extensions.
 */

@TeleOp(name = "Driver Control", group = "Robot")
//@Disabled
public class DriverControl extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    public RobotHardware robot = new RobotHardware(this);
    private final Lift lift = new Lift(robot);
    private final Arm arm = new Arm(robot);
    private final Claw claw = new Claw(robot);
    private final Bucket bucket = new Bucket(robot);


    // Use the new FtcLib gamepad extension.
    GamepadEx gamepadOne = null;
    GamepadEx gamepadTwo = null;

    @Override
    public void runOpMode() {
        gamepadOne = new GamepadEx(gamepad1);
        gamepadTwo = new GamepadEx(gamepad2);
        robot.init();
        lift.init();
        arm.init();
        claw.init();
        bucket.init();


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamepadOne.readButtons();
            gamepadTwo.readButtons();

            // Driver gamepad (A on the driver hub)
            arm.setProperties(gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                    gamepadOne.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT));

            claw.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER));

            // Arm driver gamepad (B on the driver hub)
            lift.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.A),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.X),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.Y));

            bucket.setProperties(gamepadTwo.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    gamepadTwo.wasJustPressed(GamepadKeys.Button.DPAD_UP));

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepadOne.getLeftY();  // Note: pushing stick forward gives negative value
            double lateral = gamepadOne.getLeftX();
            double yaw = gamepadOne.getRightX();
            robot.moveRobot(axial, lateral, yaw);

            lift.update();
            arm.update();
            claw.update();
            bucket.update();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Lift State", lift.liftState);
            telemetry.addData("Lift Target", "%d", robot.liftMotor.getTargetPosition());
            telemetry.addData("Lift Position", "%d", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Lift Power", "%6.2f", robot.liftMotor.getPower());
            telemetry.addData("Lift Busy", robot.liftMotor.isBusy());
            telemetry.addData("Lift Mode", robot.liftMotor.getMode());
            telemetry.addData("Lift PIDF Run To Position", robot.liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.addData("Arm State", arm.armState);
            telemetry.addData("Arm Target", "%d", robot.armMotor.getTargetPosition());
            telemetry.addData("Arm position", "%d", robot.armMotor.getCurrentPosition());
            telemetry.addData("Arm Power", "%6.2f", robot.armMotor.getPower());
            telemetry.addData("Arm Busy", robot.armMotor.isBusy());
            telemetry.addData("Arm Mode", robot.armMotor.getMode());
            telemetry.addData("Arm PIDF Run To Position", robot.armMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.update();


        }
    }
}
