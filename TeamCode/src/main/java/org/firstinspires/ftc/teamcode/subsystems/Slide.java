package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.SlidePosition;

public class Slide extends SubSystem {
    public SlidePosition slideState;
    public DcMotorEx slideMotor;
    private RobotHardware robot;
    public double slidePower = 0;
    private final int SLIDE_MAX = -100;

    private final double SLIDE_MAX_POWER = .7;
    private final int SLIDE_POSITION_TOLERANCE = 10;
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
        int currentPosition = slideMotor.getCurrentPosition();
        if (currentPosition > SLIDE_MAX && slidePower > 0) {
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
