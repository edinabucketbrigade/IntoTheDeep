/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * Perform encoder tests on a motor. Implemented as an iterative OpMode.
 */
@TeleOp(name = "Encoder Test", group = "Test")
//@Disabled
public class EncoderTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private GamepadEx gamePad;
    private DcMotorEx motor;
    private int POSITION_TOLERANCE = 20;

    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Dpad Up and Dpad Down");
        telemetry.update();
        gamePad = new GamepadEx(gamepad1);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        // Make sure the motor is stopped and encoder is set to 0;
        stopAndResetEncoder(motor);
        // Use encoders to enable manual movement of the motor to deternmine position values.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfVelocityCoefficients = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfPositionCoefficients = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        // These values are recommended as a starting point if you are tuning a PID.
        pidfVelocityCoefficients.p = 1.063f;
        pidfVelocityCoefficients.i = 1.063f;
        pidfVelocityCoefficients.f = 10.63f;
        motor.setVelocityPIDFCoefficients(pidfVelocityCoefficients.p, pidfVelocityCoefficients.i, pidfVelocityCoefficients.d, pidfVelocityCoefficients.f);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfPositionCoefficients);
        motor.setPositionPIDFCoefficients(10f);
        motor.setTargetPositionTolerance(5);
    }

    /**
     * This method will be called repeatedly during the period between when
     * the init button is pressed and when the play button is pressed (or the
     * OpMode is stopped).
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Dpad Up and Dpad Down");
        telemetry.addLine("Y - Stop and Reset");
        telemetry.addLine("Position is monitored real-time.");
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.update();
    }

    /**
     * This method will be called once, when the play button is pressed.
     */
    @Override
    public void start() {
        runtime.reset();
        motor.setPower(0);
    }

    /**
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        gamePad.readButtons();
        telemetry.addData("Status", "Run Time: " + runtime);
        // The actual increment and speed will depend on the motor you are testing.
        float ENCODER_INCREMENT = 769f;
        double MAX_VELOCITY = 2900;
        double RUN_VELOCITY = MAX_VELOCITY * .7f;
        if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            motor.setTargetPosition((int) (motor.getCurrentPosition() + ENCODER_INCREMENT));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(RUN_VELOCITY);
        }

        if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            motor.setTargetPosition((int) (motor.getCurrentPosition() - ENCODER_INCREMENT));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(RUN_VELOCITY);
        }

        if (gamePad.wasJustPressed(GamepadKeys.Button.Y)) {
            stopAndResetEncoder(motor);
        }

        telemetry.addData("Target", "%d", motor.getTargetPosition());
        telemetry.addData("Position", "%d", motor.getCurrentPosition());
        telemetry.addData("Toleramce error", Math.abs(motor.getCurrentPosition() - POSITION_TOLERANCE));
        telemetry.addData("Velocity", "%6.2f", motor.getVelocity());
        telemetry.addData("Power", "%6.2f", motor.getPower());
        telemetry.addData("Busy", motor.isBusy());
        telemetry.addData("Current (milli amps)", "%6.2f", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Mode", motor.getMode());
        telemetry.addData("PIDF Run Using", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("PIDF Run To Position", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.update();
    }

    /**
     * This method will be called once, when this OpMode is stopped.
     * <p>
     * Your ability to control hardware from this method will be limited.
     */
    @Override
    public void stop() {
        stopAndResetEncoder(motor);
    }

    /**
     * This seems to be the only way to reliably stop a motor and reset the encoder.
     * This wos only tested on a goBilda motor.
     *
     * @param motor Motor to stop.
     */
    private void stopAndResetEncoder(DcMotorEx motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
