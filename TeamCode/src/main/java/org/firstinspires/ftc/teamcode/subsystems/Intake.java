package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake extends SubSystem {
    private final RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    public boolean triggersPressed = false;
    // .7 to 1 seem to give the same speed. Maybe .8 saves some overheating?
    private final double MAX_SERVO_SPEED = .8;
    private final double SERVO_STOP = .5;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        robot.intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeServo.setPower(SERVO_STOP);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if (triggersPressed) {
            robot.intakeServo.setPower(SERVO_STOP);
        }

        if (leftBumperPressed) {
            robot.intakeServo.setPower(-MAX_SERVO_SPEED);
        }

        if (rightBumperPressed) {
            robot.intakeServo.setPower(MAX_SERVO_SPEED);
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper, boolean triggersPressed) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
        this.triggersPressed = triggersPressed;
    }
}
