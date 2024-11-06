package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ArmPosition;

public class Arm extends SubSystem {
    public ArmPosition armState;
    private RobotHardware robot;
    public boolean rightBumperPressed = false;
    public boolean leftBumperPressed = false;
    private final int ARM_FRONT = 0;
    private final int ARM_BACK = 384;

    private final double ARM_MAX_POWER = .7;
    private final int ARM_POSITION_TOLERANCE = 10;

    public Arm(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        armState = ArmPosition.Back;
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        switch (armState) {
            case Back:
                if (Math.abs(robot.armMotor.getCurrentPosition() - ARM_BACK) < ARM_POSITION_TOLERANCE) {
                    if (leftBumperPressed) {
                        robot.armMotor.setTargetPosition(ARM_BACK);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Front;
                    }
                }

            case Front:
                if (Math.abs(robot.armMotor.getCurrentPosition() - ARM_FRONT) < ARM_POSITION_TOLERANCE) {
                    if (rightBumperPressed) {
                        robot.armMotor.setTargetPosition(ARM_FRONT);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Back;
                    }
                }
                break;
            default:
                // if get here, there is a problem
                robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armMotor.setPower(0);
                armState = ArmPosition.Front;
        }

    }

    public void setProperties(boolean leftBumper, boolean rightBumper) {
        leftBumperPressed = leftBumper;
        rightBumperPressed = rightBumper;
    }
}