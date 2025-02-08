
package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.enums.RotatePosition;

public class Rotate extends SubSystem {

    public RotatePosition rotateState;
    public Servo rotate;
    private RobotHardware robot;
    public boolean DPAD_RIGHT = false;
    public boolean DPAD_LEFT = false;
    private final double ROTATE_OUT = 1;
    private final double ROTATE_IN = 0.375;

    public Rotate(RobotHardware robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        rotate = robot.rotateServo;
        rotateState = RotatePosition.Out;
        rotate.setPosition(ROTATE_OUT);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        switch (rotateState) {
            case Out:
                if (DPAD_LEFT) {
                    rotate.setPosition(ROTATE_IN);
                    rotateState = RotatePosition.In;
                }
            case In:
                if (DPAD_RIGHT) {
                    rotate.setPosition(ROTATE_OUT);
                    rotateState = RotatePosition.Out;
                }
        }
    }

    public class RotateDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_OUT);
            rotateState = RotatePosition.Out;
            return false;
        }
    }

    public Action rotateOut() {
        return new RotateDown();
    }

    public class BucketUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rotate.setPosition(ROTATE_IN);
            rotateState = RotatePosition.In;
            return false;
        }
    }

    public Action rotateIn() {
        return new BucketUp();
    }

    public void setProperties(boolean dpadRight, boolean dpadLeft) {
        this.DPAD_LEFT = dpadLeft;
        this.DPAD_RIGHT = dpadRight;
    }
}