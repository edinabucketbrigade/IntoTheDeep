package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.enums.ArmPosition.Back;
import static org.firstinspires.ftc.teamcode.enums.ArmPosition.Neutral;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.ArmPosition;

public class Arm extends SubSystem {
    public ArmPosition armState;
    public DcMotorEx armMotor;
    private RobotHardware robot;
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
        armMotor = robot.armMotor;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        switch (armState) {
            case Back:
                if (Math.abs(armMotor.getCurrentPosition() - ARM_BACK) < ARM_POSITION_TOLERANCE) {
                    if (DPAD_UP) {
                        armMotor.setTargetPosition(ARM_FRONT);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Front;
                    }
                    if (DPAD_RIGHT) {
                        armMotor.setTargetPosition(ARM_NEUTRAL);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = Neutral;
                    }
                }
                break;

            case Front:
                if (Math.abs(armMotor.getCurrentPosition() - ARM_FRONT) < ARM_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        armMotor.setTargetPosition(ARM_BACK);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Back;
                    }
                    if (DPAD_RIGHT) {
                        armMotor.setTargetPosition(ARM_NEUTRAL);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = Neutral;
                    }
                }
                break;

            case Neutral:
                if (Math.abs(armMotor.getCurrentPosition() - ARM_NEUTRAL) < ARM_POSITION_TOLERANCE) {
                    if (DPAD_DOWN) {
                        armMotor.setTargetPosition(ARM_BACK);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Back;
                    }
                    if (DPAD_UP) {
                        armMotor.setTargetPosition(ARM_FRONT);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(ARM_MAX_POWER);
                        armState = ArmPosition.Front;
                    }
                }
                break;

            default:
                // if get here, there is a problem
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setPower(0);
                armState = ArmPosition.Front;
        }
    }

    public class ArmNeutral implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setTargetPosition(ARM_NEUTRAL);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_MAX_POWER);
                armState = Neutral;
                initialized = true;
            }
            double currentPosition = armMotor.getCurrentPosition();
            packet.put("Arm position", currentPosition);
            if (Math.abs(currentPosition - ARM_NEUTRAL) < ARM_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action armNeutral() {
        return new ArmNeutral();
    }

    public class ArmBack implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setTargetPosition(ARM_BACK);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_MAX_POWER);
                armState = Back;
                initialized = true;
            }
            double currentPosition = armMotor.getCurrentPosition();
            packet.put("Arm position", currentPosition);
            if (Math.abs(currentPosition - ARM_BACK) < ARM_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action armBack() {
        return new ArmBack();
    }

    public class ArmFront implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setTargetPosition(ARM_FRONT);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(ARM_MAX_POWER);
                armState = Back;
                initialized = true;
            }
            double currentPosition = armMotor.getCurrentPosition();
            packet.put("Arm position", currentPosition);
            if (Math.abs(currentPosition - ARM_FRONT) < ARM_POSITION_TOLERANCE) {
                return false;
            } else {
                return true;
            }
        }
    }

    public Action armFront() {
        return new ArmFront();
    }

    public void setProperties(boolean dpadDown, boolean dpadUp, boolean dpadNeutral) {
        DPAD_DOWN = dpadDown;
        DPAD_UP = dpadUp;
        DPAD_RIGHT = dpadNeutral;
    }
}
