package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.enums.SlidePosition.Back;
import static org.firstinspires.ftc.teamcode.enums.SlidePosition.Neutral;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.SlidePosition;

public class Slide extends SubSystem {
    public SlidePosition slideState;
    public DcMotorEx slideMotor;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    public boolean DPAD_RIGHT = false;
    private final int SLIDE_FRONT = -1650;
    private final int SLIDE_NEUTRAL = -844;
    private final int SLIDE_BACK = 0;

    private final double SLIDE_MAX_POWER = .7;
    private final int SLIDE_POSITION_TOLERANCE = 10;

    public Slide(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        slideState = SlidePosition.Back;
        slideMotor = robot.armMotor;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        switch (slideState) {
            case Back:
                if (Math.abs(slideMotor.getCurrentPosition() - SLIDE_BACK) < SLIDE_POSITION_TOLERANCE) {
                    if (DPAD_UP) {
                        slideMotor.setTargetPosition(SLIDE_FRONT);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = SlidePosition.Front;
                    }
                    if (DPAD_RIGHT) {
                        slideMotor.setTargetPosition(SLIDE_NEUTRAL);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = Neutral;
                    }
                }
                break;

            case Front:
                if (Math.abs(slideMotor.getCurrentPosition() - SLIDE_FRONT) < SLIDE_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        slideMotor.setTargetPosition(SLIDE_BACK);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = SlidePosition.Back;
                    }
                    if (DPAD_RIGHT) {
                        slideMotor.setTargetPosition(SLIDE_NEUTRAL);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = Neutral;
                    }
                }
                break;

            case Neutral:
                if (Math.abs(slideMotor.getCurrentPosition() - SLIDE_NEUTRAL) < SLIDE_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        slideMotor.setTargetPosition(SLIDE_BACK);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = SlidePosition.Back;
                    }
                    if (DPAD_UP) {
                        slideMotor.setTargetPosition(SLIDE_FRONT);
                        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideMotor.setPower(SLIDE_MAX_POWER);
                        slideState = SlidePosition.Front;
                    }
                }
                break;

            default:
                // if get here, there is a problem
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setPower(0);
                slideState = SlidePosition.Front;
        }
    }

    public class SlideNeutral implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideMotor.setTargetPosition(SLIDE_NEUTRAL);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(SLIDE_MAX_POWER);
                slideState = Neutral;
                initialized = true;
            }
            double currentPosition = slideMotor.getCurrentPosition();
            packet.put("Slide position", currentPosition);
            if (Math.abs(currentPosition - SLIDE_NEUTRAL) < SLIDE_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action SlideNeutral() {
        return new SlideNeutral();
    }

    public class SlideBack implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideMotor.setTargetPosition(SLIDE_BACK);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(SLIDE_MAX_POWER);
                slideState = Back;
                initialized = true;
            }
            double currentPosition = slideMotor.getCurrentPosition();
            packet.put("Slide position", currentPosition);
            if (Math.abs(currentPosition - SLIDE_BACK) < SLIDE_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action SlideBack() {
        return new SlideBack();
    }

    public class SlideFront implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideMotor.setTargetPosition(SLIDE_FRONT);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(SLIDE_MAX_POWER);
                slideState = Back;
                initialized = true;
            }
            double currentPosition = slideMotor.getCurrentPosition();
            packet.put("Slide position", currentPosition);
            if (Math.abs(currentPosition - SLIDE_FRONT) < SLIDE_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action SlideFront() {
        return new SlideFront();
    }

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
    }
}
