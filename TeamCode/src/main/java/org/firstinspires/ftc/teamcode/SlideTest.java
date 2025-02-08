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

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.SlidePosition;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

/*
 * Test slide control.
 */
@TeleOp(name = "Slide Test", group = "Test")
//@Disabled
public class SlideTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx slideMotor = null;
    public Encoder slideEncoder = null;
    private SlideT slide = null;
    GamepadEx gamepadOne = null;
    TriggerReader triggerReaderLeft;
    TriggerReader triggerReaderRight;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadOne = new GamepadEx(gamepad1);
        triggerReaderLeft = new TriggerReader(gamepadOne, GamepadKeys.Trigger.LEFT_TRIGGER);
        triggerReaderRight = new TriggerReader(gamepadOne, GamepadKeys.Trigger.RIGHT_TRIGGER);

        slideMotor = hardwareMap.get(DcMotorEx.class, "motor");
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfVelocityCoefficients = slideMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        // These values are recommended as a starting point if you are tuning a PID.
        pidfVelocityCoefficients.p = 1.063f;
        pidfVelocityCoefficients.i = 1.063f;
        pidfVelocityCoefficients.f = 10.63f;
        slideMotor.setVelocityPIDFCoefficients(pidfVelocityCoefficients.p, pidfVelocityCoefficients.i, pidfVelocityCoefficients.d, pidfVelocityCoefficients.f);
        stopAndResetEncoder(slideMotor);
        slideEncoder = new OverflowEncoder(new RawEncoder(slideMotor));
        slideEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide = new SlideT(slideMotor);
        slide.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            gamepadOne.readButtons();
            triggerReaderLeft.readValue();
            triggerReaderRight.readValue();

            slide.setProperties(-gamepadOne.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) +
                    gamepadOne.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            slide.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Slide State", slide.slideState);
            telemetry.addData("Slide Target", "%d", slideMotor.getTargetPosition());
            telemetry.addData("Slide position", "%d", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Power", "%6.2f", slideMotor.getPower());
            telemetry.addData("Slide Busy", slideMotor.isBusy());
            telemetry.addData("Slide Mode", slideMotor.getMode());
            telemetry.addData("Slide PIDF Run To Position", slideMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }

    public class SlideT extends SubSystem {
        public SlidePosition slideState;
        public DcMotorEx slideMotor;
        private RobotHardware robot;
        public double slidePower = 0;
        private final int SLIDE_MAX = 2000;

        public SlideT(DcMotorEx slideMotor) {
            this.slideMotor = slideMotor;
        }

        @Override
        public void init() {
            slideState = SlidePosition.Back;
            slideMotor.setPower(0);
        }

        @Override
        public void start() {

        }

        @Override
        public void update() {
            int currentPosition = slideMotor.getCurrentPosition();
            if (currentPosition < SLIDE_MAX && slidePower > 0) {
                slideMotor.setPower(slidePower);
            } else if (currentPosition >= 0 && slidePower < 0) {
                slideMotor.setPower(slidePower);
            } else {
                slideMotor.setPower(0);
            }
        }

        public void setProperties(double slidePower) {
            this.slidePower = slidePower;
        }
    }

    /**
     * This seems to be the only way to reliably stop a motor and reset the encoder.
     * This wos only tested on a goBilda motor.
     *
     * @param motor Motor to stop.
     */
    public void stopAndResetEncoder(DcMotorEx motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
