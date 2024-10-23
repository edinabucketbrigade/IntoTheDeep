package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.LiftPosition;

public class Lift extends SubSystem {
    private LiftPosition liftPosition;
    private LiftPosition liftState;
    private RobotHardware robot;
    private boolean aPressed = false;
    private boolean xPressed = false;
    private boolean yPressed = false;

    // Encoder positions for the lift.
    //TODO Test to find the proper values.
    private final int LIFT_DOWN = 0;
    private final int LIFT_LOW = 10;
    private final int LIFT_HIGH = 20;

    private final double LIFT_MAX_POWER = .7;

    public Lift(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        liftPosition = LiftPosition.Down;
        liftState = liftPosition;
        // Make sure encoder is 0 at start.
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (liftState) {
            case Down:
                if (robot.liftMotor.getCurrentPosition() < 5) {
                    if (xPressed) {
                        robot.liftMotor.setTargetPosition(LIFT_DOWN);
                        liftState = LiftPosition.LowBasket;
                    }

                    if (yPressed) {
                        robot.liftMotor.setTargetPosition(LIFT_HIGH);
                        liftState = LiftPosition.HighBasket;
                    }
                }
                break;
            case LowBasket:
                if (robot.liftMotor.getCurrentPosition() < 5) {
                    if (aPressed) {
                        robot.liftMotor.setTargetPosition(LIFT_DOWN);
                        liftState = LiftPosition.Down;
                    }

                    if (yPressed) {
                        robot.liftMotor.setTargetPosition(LIFT_HIGH);
                        liftState = LiftPosition.HighBasket;
                    }
                }
                break;
            case HighBasket:
                if (robot.liftMotor.getCurrentPosition() < 5) {
                    if (aPressed) {
                        robot.liftMotor.setTargetPosition(LIFT_DOWN);
                        liftState = LiftPosition.Down;
                    }
                }
                break;
            default:
                // If we get here we have problem, stop the lift.
                robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotor.setPower(0);
                liftState = LiftPosition.Down;
        }

        robot.liftMotor.setPower(LIFT_MAX_POWER);
    }

    // Respond to gamepad inputs.
    public void setProperties(boolean buttonA, boolean buttonX, boolean buttonY) {
        if (buttonA) {
            liftPosition = LiftPosition.Down;
        } else if (buttonX) {
            liftPosition = LiftPosition.LowBasket;
        } else if (buttonY) {
            liftPosition = LiftPosition.HighBasket;
        }
    }
}
