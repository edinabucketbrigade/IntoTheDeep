package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.LiftPosition;

public class Lift extends SubSystem {
    LiftPosition liftPosition;
    LiftPosition liftState;
    RobotHardware robot;
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
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (robot.liftMotor.isBusy() && (liftState != liftPosition)) {
            return;
        } else {
            liftState = liftPosition;
        }

        // Only try to move if we are finished with any moves.
        if (!robot.liftMotor.isBusy()) {
            switch (liftPosition) {
                case Down:
                    robot.liftMotor.setTargetPosition(LIFT_DOWN);
                case LowBasket:
                    robot.liftMotor.setTargetPosition(LIFT_LOW);
                case HighBasket:
                    robot.liftMotor.setTargetPosition(LIFT_HIGH);
            }

            if (liftState != liftPosition) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(LIFT_MAX_POWER);
            }
        }
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
