package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ArmPosition;
import org.firstinspires.ftc.teamcode.enums.ClawPosition;

public class Claw extends SubSystem {

    public ClawPosition clawState;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final int CLAW_CLOSED = 0;
    private final int CLAW_OPEN = 90;

    private final double CLAW_MAX_POWER = .7;

    public Claw(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        clawState = ClawPosition.Open;
        robot.clawServo.setPosition(0);


    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }
}
