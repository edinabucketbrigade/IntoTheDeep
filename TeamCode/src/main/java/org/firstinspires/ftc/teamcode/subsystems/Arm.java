package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;

import static org.firstinspires.ftc.teamcode.enums.ArmPosition.Neutral;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ArmPosition;

public class Arm extends SubSystem {
    public ArmPosition armState;
    private RobotHardware robot;
    private Lift lift;
    public boolean DPAD_UP = false;
    public boolean DPAD_DOWN = false;
    public boolean DPAD_RIGHT = false;
    private final int ARM_FRONT = -1650;
    private final int ARM_NEUTRAL = -844;
    private final int ARM_BACK = 0;

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
                    if (DPAD_UP) {
                        robot.armMotor.setTargetPosition(ARM_FRONT);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Front;
                    }
                    if (DPAD_RIGHT) {
                        robot.armMotor.setTargetPosition(ARM_NEUTRAL);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = Neutral;
                    }
                }

            case Front:
                if (Math.abs(robot.armMotor.getCurrentPosition() - ARM_FRONT) < ARM_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        robot.armMotor.setTargetPosition(ARM_BACK);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Back;
                    }
                    if (DPAD_RIGHT) {
                        robot.armMotor.setTargetPosition(ARM_NEUTRAL);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = Neutral;
                    }
                }

            case Neutral:
                if (Math.abs(robot.armMotor.getCurrentPosition() - ARM_NEUTRAL) < ARM_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        robot.armMotor.setTargetPosition(ARM_BACK);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Back;
                    }
                    if (DPAD_UP) {
                        robot.armMotor.setTargetPosition(ARM_FRONT);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Front;
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

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
    }
}
