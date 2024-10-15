package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.LiftPosition;

public class Lift extends SubSystem {
    LiftPosition liftPosition;
    RobotHardware robot;
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
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if (!robot.liftMotor.isBusy()) {
            switch (liftPosition) {
                case Down:
                    robot.liftMotor.setTargetPosition(LIFT_DOWN);
                case LowBasket:
                    robot.liftMotor.setTargetPosition(LIFT_LOW);
                case HighBasket:
                    robot.liftMotor.setTargetPosition(LIFT_HIGH);
            }

            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor.setPower(LIFT_MAX_POWER);
        }
    }

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