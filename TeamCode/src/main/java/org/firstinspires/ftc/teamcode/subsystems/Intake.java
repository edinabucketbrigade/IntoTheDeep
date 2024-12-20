package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake extends SubSystem {
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    public boolean DPADrightPressed = false;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        robot.intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeServo.setPower(0);
    }

    @Override
    public void start() {

    }

    //TODO: Change power values and add trigger code to stop servo.
    // On a CRServo 0 stops, <0 is reverse, >0 is forward.
    @Override
    public void update() {
        if (leftBumperPressed) {
            robot.intakeServo.setPower(-1);
        }
        if (rightBumperPressed) {
            robot.intakeServo.setPower(1);
        }
        if (DPADrightPressed) {
            robot.intakeServo.setPower(0);
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper, boolean buttonB) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
        DPADrightPressed = buttonB;
    }
}
