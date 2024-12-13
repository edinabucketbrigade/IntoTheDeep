package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class Intake extends SubSystem {
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;

    public Intake(RobotHardware robot){
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

    @Override
    public void update() {
    if(leftBumperPressed){
        robot.intakeServo.setPower(0);
    }if(rightBumperPressed){
        robot.intakeServo.setPower(1);
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}
