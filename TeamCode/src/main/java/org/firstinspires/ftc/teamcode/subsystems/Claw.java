package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ClawPosition;

public class Claw extends SubSystem {

    public ClawPosition clawState;
    public Servo claw;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final double CLAW_CLOSED = 1;
    private final double CLAW_OPEN = 0.5;

    public Claw(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        clawState = ClawPosition.Close;
        claw = robot.clawServo;
        claw.setPosition(CLAW_CLOSED);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (clawState) {
            case Close:
                if (leftBumperPressed) {
                    claw.setPosition(CLAW_CLOSED);
                    clawState = ClawPosition.Open;
                }
            case Open:
                if (rightBumperPressed) {
                    claw.setPosition(CLAW_OPEN);
                    clawState = ClawPosition.Close;
                }
        }
    }


    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}