package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;
//TODO: Add RR Action code if this is going to by used by RR in the new robot.
public class Intake extends SubSystem {
    private final RobotHardware robot;
    private CRServo intakeServo;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    public boolean triggersPressed = false;
    // .7 to 1 seem to give the same speed. Maybe .8 saves some overheating?
    private final double MAX_SERVO_SPEED = .8;
    private final double SERVO_STOP = .5;
    public boolean DPADrightPressed = false;

    public Intake(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        intakeServo = robot.intakeServo;
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(SERVO_STOP);
    }

    @Override
    public void start() {

    }

    //TODO: Change power values and add trigger code to stop servo.
    // On a CRServo 0 stops, <0 is reverse, >0 is forward.
    @Override
    public void update() {
        if (triggersPressed) {
            intakeServo.setPower(SERVO_STOP);
        }

        if (leftBumperPressed) {
            intakeServo.setPower(-MAX_SERVO_SPEED);
        }

        if (rightBumperPressed) {
            intakeServo.setPower(MAX_SERVO_SPEED);
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper, boolean triggersPressed) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
        this.triggersPressed = triggersPressed;
    }
}
