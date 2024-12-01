package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ArmPosition;
import org.firstinspires.ftc.teamcode.enums.ClawPosition;

public class Claw extends SubSystem {

    public ClawPosition clawState;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final double CLAW_CLOSED = 0;
    private final double CLAW_OPEN = 1;

    public Claw(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        clawState = ClawPosition.Close;
        robot.clawServo.setPosition(CLAW_CLOSED);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (clawState) {
            case Close:
                if (leftBumperPressed) {
                    robot.clawServo.setPosition(CLAW_CLOSED);
                    clawState = ClawPosition.Open;
                }
            case Open:
                if (rightBumperPressed) {
                    robot.clawServo.setPosition(CLAW_OPEN);
                    clawState = ClawPosition.Close;
                }
        }
    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}