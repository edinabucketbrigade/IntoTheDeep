package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.enums.ArmPosition.Neutral;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.LiftPosition;

public class Lift extends SubSystem {
    public LiftPosition liftState;
    public DcMotorEx lift;
    private RobotHardware robot;
    public boolean aPressed = false;
    public boolean xPressed = false;
    public boolean yPressed = false;

    // Encoder positions for the lift.
    //TODO Test to find the proper values.
    private final int LIFT_DOWN = 0;
    private final int LIFT_LOW = -797;

    private final int LIFT_HIGH = -1866;

    private final double LIFT_MAX_POWER = .7;
    private final int LIFT_POSITION_TOLERANCE = 10;

    public Lift(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        liftState = LiftPosition.Down;
        lift = robot.liftMotor;
        // Make sure encoder is 0 at start
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (liftState) {
            case Down:
                if (Math.abs(lift.getCurrentPosition() - LIFT_DOWN) < LIFT_POSITION_TOLERANCE) {
                    if (xPressed) {
                        lift.setTargetPosition(LIFT_LOW);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(LIFT_MAX_POWER);
                        liftState = LiftPosition.LowBasket;
                    }

                    if (yPressed) {
                        lift.setTargetPosition(LIFT_HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(LIFT_MAX_POWER);
                        liftState = LiftPosition.HighBasket;
                    }
                }
                break;
            case LowBasket:
                if (Math.abs(lift.getCurrentPosition() - LIFT_LOW) < LIFT_POSITION_TOLERANCE) {
                    if (aPressed) {
                        lift.setTargetPosition(LIFT_DOWN);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(LIFT_MAX_POWER);
                        liftState = LiftPosition.Down;
                    }

                    if (yPressed) {
                        lift.setTargetPosition(LIFT_HIGH);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(LIFT_MAX_POWER);
                        liftState = LiftPosition.HighBasket;
                    }
                }
                break;
            case HighBasket:
                if (Math.abs(lift.getCurrentPosition() - LIFT_HIGH) < LIFT_POSITION_TOLERANCE) {
                    if (aPressed) {
                        lift.setTargetPosition(LIFT_DOWN);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(LIFT_MAX_POWER);
                        liftState = LiftPosition.Down;
                    }
                }
                break;
            default:
                // if get here, there is a problem
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setPower(0);
                liftState = LiftPosition.Down;
        }

    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setTargetPosition(LIFT_DOWN);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(LIFT_MAX_POWER);
                liftState = LiftPosition.Down;
                initialized = true;
            }
            double currentPosition = lift.getCurrentPosition();
            packet.put("Lift position", currentPosition);
            if (Math.abs(currentPosition - LIFT_DOWN) < LIFT_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class LiftLow implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setTargetPosition(LIFT_LOW);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(LIFT_MAX_POWER);
                liftState = LiftPosition.LowBasket;
                initialized = true;
            }
            double currentPosition = lift.getCurrentPosition();
            packet.put("Lift position", currentPosition);
            if (Math.abs(currentPosition - LIFT_LOW) < LIFT_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class LiftHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setTargetPosition(LIFT_HIGH);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(LIFT_MAX_POWER);
                liftState = LiftPosition.HighBasket;
                initialized = true;
            }
            double currentPosition = lift.getCurrentPosition();
            packet.put("Lift position", currentPosition);
            if (Math.abs(currentPosition - LIFT_HIGH) < LIFT_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    // Respond to gamepad inputs.
    public void setProperties(boolean buttonA, boolean buttonX, boolean buttonY) {
        aPressed = buttonA;
        xPressed = buttonX;
        yPressed = buttonY;
    }
}
