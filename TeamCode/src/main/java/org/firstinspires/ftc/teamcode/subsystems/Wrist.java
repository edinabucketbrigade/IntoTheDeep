package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.WristPosition;

public class Wrist extends SubSystem {

    public WristPosition wristState;
    public Servo wrist;
    private RobotHardware robot;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    public boolean DPAD_RIGHT = false;
    private final double WRIST_DOWN = 1;
    private final double WRIST_NEUTRAL = 0.5;
    private final double WRIST_UP = 0;

    public Wrist(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        wristState = WristPosition.Neutral;
        wrist = robot.wristServo;
        wrist.setPosition(WRIST_NEUTRAL);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
    }
}
