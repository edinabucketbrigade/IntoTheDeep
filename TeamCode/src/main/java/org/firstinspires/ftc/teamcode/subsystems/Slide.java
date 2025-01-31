package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.SlidePosition;

public class Slide extends SubSystem {
    public SlidePosition slideState;
    public DcMotorEx slideMotor;
    private RobotHardware robot;
    public double slidePower = 0;
    private final int SLIDE_MAX = -1370;

    private final double SLIDE_MAX_POWER = .7;
    private final int SLIDE_POSITION_TOLERANCE = 10;

    public Slide(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        slideState = SlidePosition.Back;
        slideMotor = robot.slideMotor;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        int currentPosition = slideMotor.getCurrentPosition();
        if (currentPosition > SLIDE_MAX) {
            slideMotor.setPower(slidePower);
        }
    }

    public void setProperties(double slidePower) {
        this.slidePower = slidePower;
    }
}
