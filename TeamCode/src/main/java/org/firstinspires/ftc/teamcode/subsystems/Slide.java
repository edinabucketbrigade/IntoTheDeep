package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.SlidePosition;

public class Slide extends SubSystem {
    public SlidePosition slideState;
    public DcMotorEx slideMotor;
    private RobotHardware robot;
    public double slidePower = 0;
    private final int SLIDE_MAX = 1320;
    private int currentPosition;
    public boolean extended() {
        if (currentPosition > 700) return true;
        else return false;
    }

//-1344
    public Slide(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        slideState = SlidePosition.Back;
        slideMotor = robot.slideMotor;
        slideMotor.setPower(0);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        currentPosition = slideMotor.getCurrentPosition();
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
